// Switch GamePad : transforme la Switch en GamePad Wii U pour Cemu.
// - Manette (boutons, sticks, gyroscope) : protocole DSU en UDP, port 26760.
// - Ecran du GamePad : images JPEG recues du PC en TCP, port 26761 (programme compagnon).
// - Tactile : les touchers sont renvoyes au PC, qui les transforme en clics dans la fenetre du GamePad.

#include <switch.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <turbojpeg.h>

#define DSU_PORT		26760
#define VIDEO_PORT		26761
#define DSU_VERSION		1001
#define MAX_CLIENTS		4
#define CLIENT_TIMEOUT_NS	5000000000ULL	// un client qui ne redemande rien pendant 5 s est oublie

#define MSG_VERSION	0x100000
#define MSG_PORTS	0x100001
#define MSG_DATA	0x100002

#define SCREEN_W	1280
#define SCREEN_H	720
#define MAX_JPEG_SIZE	(2 * 1024 * 1024)

// Zone morte des sticks (fraction de la course) : neutralise le drift des Joy-Con.
#define STICK_DEADZONE	0.30f

// Sens des axes : passe une valeur a -1.0f si un mouvement est inverse dans Cemu.
#define GYRO_PITCH_SIGN	1.0f
#define GYRO_YAW_SIGN	1.0f
#define GYRO_ROLL_SIGN	1.0f
#define ACCEL_X_SIGN	1.0f
#define ACCEL_Y_SIGN	1.0f
#define ACCEL_Z_SIGN	1.0f

// Un client DSU (Cemu) qui a demande a recevoir les donnees.
typedef struct {
	bool			used;		// emplacement occupe ou non
	struct sockaddr_in	addr;		// adresse IP + port du PC
	u64			last_request;	// instant de la derniere demande (ns)
} Client;

// Etat partage entre la boucle principale et le fil qui recoit l'image.
static Mutex		g_frame_lock;
static u8		*g_frame = NULL;	// derniere image decodee, RGBA 1280x720
static volatile bool	g_frame_ready = false;	// une nouvelle image attend d'etre affichee
static volatile bool	g_running = true;	// passe a false pour arreter le fil video
static volatile int	g_video_client = -1;	// connexion TCP avec le programme compagnon
static volatile u32	g_frames_received = 0;

static u32 crc_table[256];

static void crc_init(void)
{
	for (u32 i = 0; i < 256; i++) {
		u32 c = i;
		for (int k = 0; k < 8; k++)
			c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
		crc_table[i] = c;
	}
}

static u32 crc32_calc(const u8 *data, size_t len)
{
	u32 c = 0xFFFFFFFFu;
	for (size_t i = 0; i < len; i++)
		c = crc_table[(c ^ data[i]) & 0xFF] ^ (c >> 8);
	return c ^ 0xFFFFFFFFu;
}

static void put_u16(u8 *p, u16 v) { memcpy(p, &v, 2); }
static void put_u32(u8 *p, u32 v) { memcpy(p, &v, 4); }
static void put_u64(u8 *p, u64 v) { memcpy(p, &v, 8); }
static void put_f32(u8 *p, float v) { memcpy(p, &v, 4); }

static u64 now_ns(void)
{
	return armTicksToNs(armGetSystemTick());
}

// En-tete commun a tous les paquets DSU envoyes par le serveur (16 octets).
static void write_header(u8 *pkt, size_t total_len, u32 server_id)
{
	memcpy(pkt, "DSUS", 4);
	put_u16(pkt + 4, DSU_VERSION);
	put_u16(pkt + 6, (u16)(total_len - 16));
	put_u32(pkt + 8, 0);
	put_u32(pkt + 12, server_id);
}

static void finish_packet(u8 *pkt, size_t total_len)
{
	put_u32(pkt + 8, 0);
	put_u32(pkt + 8, crc32_calc(pkt, total_len));
}

// Debut commun des reponses "ports" et "donnees" (12 octets) : decrit la manette du slot 0.
static void write_slot_info(u8 *p, u8 slot, bool connected)
{
	p[0] = slot;
	p[1] = connected ? 2 : 0;	// 2 = connectee
	p[2] = connected ? 2 : 0;	// 2 = gyroscope complet
	p[3] = connected ? 2 : 0;	// 2 = sans fil
	memset(p + 4, 0, 6);		// adresse MAC
	if (connected)
		p[9] = 0xAA;		// fausse MAC non nulle
	p[10] = connected ? 0x05 : 0;	// batterie : pleine
	p[11] = connected ? 1 : 0;	// manette active
}

// Convertit un stick (-32767..32767) en octet DSU (0..255, 128 = centre), avec zone morte.
static void stick_to_dsu(const HidAnalogStickState *s, u8 *out_x, u8 *out_y)
{
	float x = s->x / 32767.0f;
	float y = s->y / 32767.0f;
	float mag = sqrtf(x * x + y * y);
	if (mag < STICK_DEADZONE) {
		x = 0.0f;
		y = 0.0f;
	} else {
		// on redistribue la course restante pour garder toute la precision
		float scale = (mag - STICK_DEADZONE) / (1.0f - STICK_DEADZONE) / mag;
		x *= scale;
		y *= scale;
	}
	if (x > 1.0f) x = 1.0f;
	if (x < -1.0f) x = -1.0f;
	if (y > 1.0f) y = 1.0f;
	if (y < -1.0f) y = -1.0f;
	*out_x = (u8)(128.0f + x * 127.0f);
	*out_y = (u8)(128.0f + y * 127.0f);
}

// Attend que la socket soit lisible, en verifiant regulierement si on doit s'arreter.
static bool wait_readable(int fd)
{
	while (g_running) {
		fd_set set;
		FD_ZERO(&set);
		FD_SET(fd, &set);
		struct timeval tv = { 0, 100000 };	// 100 ms
		int r = select(fd + 1, &set, NULL, NULL, &tv);
		if (r > 0)
			return true;
		if (r < 0)
			return false;
	}
	return false;
}

// Recoit exactement len octets (ou echoue).
static bool recv_all(int fd, u8 *buf, size_t len)
{
	size_t done = 0;
	while (done < len) {
		if (!wait_readable(fd))
			return false;
		ssize_t n = recv(fd, buf + done, len - done, 0);
		if (n <= 0)
			return false;
		done += (size_t)n;
	}
	return true;
}

// Fil qui recoit les images JPEG du PC et les decode.
static void video_thread(void *arg)
{
	(void)arg;
	int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
	if (listen_fd < 0)
		return;
	int yes = 1;
	setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
	struct sockaddr_in local;
	memset(&local, 0, sizeof(local));
	local.sin_family = AF_INET;
	local.sin_port = htons(VIDEO_PORT);
	local.sin_addr.s_addr = htonl(INADDR_ANY);
	if (bind(listen_fd, (struct sockaddr *)&local, sizeof(local)) < 0 || listen(listen_fd, 1) < 0) {
		close(listen_fd);
		return;
	}

	tjhandle tj = tjInitDecompress();
	u8 *jpeg = malloc(MAX_JPEG_SIZE);
	u8 *decoded = malloc(SCREEN_W * SCREEN_H * 4);

	while (g_running && tj && jpeg && decoded) {
		if (!wait_readable(listen_fd))
			break;
		int fd = accept(listen_fd, NULL, NULL);
		if (fd < 0)
			continue;
		setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
		g_video_client = fd;

		for (;;) {
			u8 hdr[4];
			if (!recv_all(fd, hdr, 4))
				break;
			u32 size;
			memcpy(&size, hdr, 4);
			if (size == 0 || size > MAX_JPEG_SIZE)
				break;
			if (!recv_all(fd, jpeg, size))
				break;

			int w = 0, h = 0, subsamp = 0, colorspace = 0;
			if (tjDecompressHeader3(tj, jpeg, size, &w, &h, &subsamp, &colorspace) != 0)
				continue;
			if (w <= 0 || h <= 0 || w > SCREEN_W || h > SCREEN_H)
				continue;
			if (w != SCREEN_W || h != SCREEN_H)
				memset(decoded, 0, SCREEN_W * SCREEN_H * 4);
			if (tjDecompress2(tj, jpeg, size, decoded, w, SCREEN_W * 4, h, TJPF_RGBA, TJFLAG_FASTDCT) != 0)
				continue;

			mutexLock(&g_frame_lock);
			memcpy(g_frame, decoded, SCREEN_W * SCREEN_H * 4);
			g_frame_ready = true;
			g_frames_received++;
			mutexUnlock(&g_frame_lock);
		}

		g_video_client = -1;
		close(fd);
	}

	free(decoded);
	free(jpeg);
	if (tj)
		tjDestroy(tj);
	close(listen_fd);
}

// Envoie un evenement tactile au PC : type, etat (1 = appuye), x, y (ecran 1280x720).
static void send_touch(bool down, u32 x, u32 y)
{
	int fd = g_video_client;
	if (fd < 0)
		return;
	u8 msg[6];
	msg[0] = 1;
	msg[1] = down ? 1 : 0;
	put_u16(msg + 2, (u16)x);
	put_u16(msg + 4, (u16)y);
	send(fd, msg, sizeof(msg), 0);
}

int main(int argc, char **argv)
{
	consoleInit(NULL);
	crc_init();
	mutexInit(&g_frame_lock);
	g_frame = calloc(SCREEN_W * SCREEN_H, 4);

	padConfigureInput(1, HidNpadStyleSet_NpadStandard);
	PadState pad;
	padInitializeDefault(&pad);
	hidInitializeTouchScreen();

	// Capteurs de mouvement : mode portable, manette Pro, Joy-Con en paire.
	HidSixAxisSensorHandle h_handheld, h_fullkey, h_dual[2];
	hidGetSixAxisSensorHandles(&h_handheld, 1, HidNpadIdType_Handheld, HidNpadStyleTag_NpadHandheld);
	hidGetSixAxisSensorHandles(&h_fullkey, 1, HidNpadIdType_No1, HidNpadStyleTag_NpadFullKey);
	hidGetSixAxisSensorHandles(h_dual, 2, HidNpadIdType_No1, HidNpadStyleTag_NpadJoyDual);
	hidStartSixAxisSensor(h_handheld);
	hidStartSixAxisSensor(h_fullkey);
	hidStartSixAxisSensor(h_dual[0]);
	hidStartSixAxisSensor(h_dual[1]);

	// Empeche la mise en veille automatique pendant qu'on joue sur le PC.
	appletSetAutoSleepDisabled(true);
	appletSetMediaPlaybackState(true);

	Result rc = socketInitializeDefault();
	int sock = -1;
	if (R_SUCCEEDED(rc)) {
		sock = socket(AF_INET, SOCK_DGRAM, 0);
		struct sockaddr_in local;
		memset(&local, 0, sizeof(local));
		local.sin_family = AF_INET;
		local.sin_port = htons(DSU_PORT);
		local.sin_addr.s_addr = htonl(INADDR_ANY);
		if (sock < 0 || bind(sock, (struct sockaddr *)&local, sizeof(local)) < 0) {
			close(sock);
			sock = -1;
		} else {
			fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK);
		}
	}

	Thread vthread;
	bool vthread_ok = false;
	if (sock >= 0 && g_frame) {
		if (R_SUCCEEDED(threadCreate(&vthread, video_thread, NULL, NULL, 0x40000, 0x2C, -2)))
			vthread_ok = R_SUCCEEDED(threadStart(&vthread));
	}

	struct in_addr ip;
	ip.s_addr = (in_addr_t)gethostid();

	printf("\x1b[2J\x1b[1;1H");
	printf("=== Switch GamePad - pour Cemu ===\n\n");
	if (sock < 0) {
		printf("ERREUR : impossible d'ouvrir le reseau.\n");
		printf("Verifie que la Switch est connectee au Wi-Fi.\n");
	} else {
		printf("Adresse IP de la Switch : %s\n\n", inet_ntoa(ip));
		printf("Manette (Cemu, Parametres des manettes) :\n");
		printf("  API DSUController, IP ci-dessus, port %d\n\n", DSU_PORT);
		printf("Ecran du GamePad : lance le programme compagnon\n");
		printf("sur le PC avec l'IP ci-dessus. L'image remplacera\n");
		printf("ce texte des la premiere image recue.\n");
	}
	printf("\nQuitter : appuyer sur + et - en meme temps.\n");

	Client clients[MAX_CLIENTS];
	memset(clients, 0, sizeof(clients));
	u32 server_id = (u32)now_ns() ^ 0x5D5A1234u;
	u32 packet_number = 0;
	u8 rx[128];
	u8 tx[128];

	// Affichage : texte (console) au depart, puis image du GamePad.
	bool video_mode = false;
	Framebuffer fb;

	// Diagnostic affiche tant qu'on est en mode texte.
	u32 stat_version = 0, stat_ports = 0, stat_data = 0;
	u32 stat_sent = 0, stat_send_err = 0;
	u64 last_screen = 0;

	// Etat du tactile au tour precedent, pour n'envoyer que les changements.
	bool touch_was_down = false;
	u32 touch_last_x = 0, touch_last_y = 0;

	while (appletMainLoop()) {
		padUpdate(&pad);
		u64 held = padGetButtons(&pad);
		if ((held & HidNpadButton_Plus) && (held & HidNpadButton_Minus))
			break;

		u64 t = now_ns();

		// 1) Lecture des demandes DSU de Cemu.
		if (sock >= 0) {
			for (;;) {
				struct sockaddr_in from;
				socklen_t fromlen = sizeof(from);
				ssize_t n = recvfrom(sock, rx, sizeof(rx), 0, (struct sockaddr *)&from, &fromlen);
				if (n < 20 || memcmp(rx, "DSUC", 4) != 0)
					break;
				u32 type;
				memcpy(&type, rx + 16, 4);

				if (type == MSG_VERSION) {
					stat_version++;
					size_t len = 24;
					memset(tx, 0, sizeof(tx));
					write_header(tx, len, server_id);
					put_u32(tx + 16, MSG_VERSION);
					put_u16(tx + 20, DSU_VERSION);
					finish_packet(tx, len);
					sendto(sock, tx, len, 0, (struct sockaddr *)&from, fromlen);
				} else if (type == MSG_PORTS && n >= 24) {
					stat_ports++;
					s32 count;
					memcpy(&count, rx + 20, 4);
					if (count < 0) count = 0;
					if (count > 4) count = 4;
					for (s32 i = 0; i < count && 24 + i < n; i++) {
						u8 slot = rx[24 + i];
						size_t len = 32;
						memset(tx, 0, sizeof(tx));
						write_header(tx, len, server_id);
						put_u32(tx + 16, MSG_PORTS);
						write_slot_info(tx + 20, slot, slot == 0);
						finish_packet(tx, len);
						sendto(sock, tx, len, 0, (struct sockaddr *)&from, fromlen);
					}
				} else if (type == MSG_DATA) {
					stat_data++;
					int free_idx = -1, found = -1;
					for (int i = 0; i < MAX_CLIENTS; i++) {
						if (clients[i].used && clients[i].addr.sin_addr.s_addr == from.sin_addr.s_addr
							&& clients[i].addr.sin_port == from.sin_port)
							found = i;
						else if (!clients[i].used && free_idx < 0)
							free_idx = i;
					}
					int idx = found >= 0 ? found : free_idx;
					if (idx >= 0) {
						clients[idx].used = true;
						clients[idx].addr = from;
						clients[idx].last_request = t;
					}
				}
			}
		}

		// 2) Lecture des commandes de la Switch.
		HidAnalogStickState ls = padGetStickPos(&pad, 0);
		HidAnalogStickState rs = padGetStickPos(&pad, 1);

		HidSixAxisSensorState six;
		memset(&six, 0, sizeof(six));
		u32 style = padGetStyleSet(&pad);
		if (style & HidNpadStyleTag_NpadHandheld)
			hidGetSixAxisSensorStates(h_handheld, &six, 1);
		else if (style & HidNpadStyleTag_NpadFullKey)
			hidGetSixAxisSensorStates(h_fullkey, &six, 1);
		else if (style & HidNpadStyleTag_NpadJoyDual)
			hidGetSixAxisSensorStates(h_dual[1], &six, 1);	// Joy-Con droit

		HidTouchScreenState touch;
		memset(&touch, 0, sizeof(touch));
		hidGetTouchScreenStates(&touch, 1);

		// 3) Tactile -> PC (clics dans la fenetre du GamePad).
		bool touch_down = touch.count > 0;
		u32 tx_pos = touch_down ? touch.touches[0].x : touch_last_x;
		u32 ty_pos = touch_down ? touch.touches[0].y : touch_last_y;
		if (touch_down != touch_was_down || (touch_down && (tx_pos != touch_last_x || ty_pos != touch_last_y)))
			send_touch(touch_down, tx_pos, ty_pos);
		touch_was_down = touch_down;
		touch_last_x = tx_pos;
		touch_last_y = ty_pos;

		// 4) Envoi des donnees DSU aux clients enregistres.
		if (sock >= 0) {
			size_t len = 100;
			memset(tx, 0, sizeof(tx));
			write_header(tx, len, server_id);
			put_u32(tx + 16, MSG_DATA);
			write_slot_info(tx + 20, 0, true);
			u8 *d = tx + 31;
			d[0] = 1;			// manette active
			put_u32(d + 1, packet_number++);

			u8 b1 = 0, b2 = 0;
			if (held & HidNpadButton_Left)		b1 |= 0x80;
			if (held & HidNpadButton_Down)		b1 |= 0x40;
			if (held & HidNpadButton_Right)		b1 |= 0x20;
			if (held & HidNpadButton_Up)		b1 |= 0x10;
			if (held & HidNpadButton_Plus)		b1 |= 0x08;
			if (held & HidNpadButton_StickR)	b1 |= 0x04;
			if (held & HidNpadButton_StickL)	b1 |= 0x02;
			if (held & HidNpadButton_Minus)		b1 |= 0x01;
			if (held & HidNpadButton_Y)		b2 |= 0x80;	// gauche
			if (held & HidNpadButton_B)		b2 |= 0x40;	// bas
			if (held & HidNpadButton_A)		b2 |= 0x20;	// droite
			if (held & HidNpadButton_X)		b2 |= 0x10;	// haut
			if (held & HidNpadButton_R)		b2 |= 0x08;
			if (held & HidNpadButton_L)		b2 |= 0x04;
			if (held & HidNpadButton_ZR)		b2 |= 0x02;
			if (held & HidNpadButton_ZL)		b2 |= 0x01;
			d[5] = b1;
			d[6] = b2;
			d[7] = 0;				// bouton HOME (non lisible)
			d[8] = 0;				// le tactile passe par le PC, pas par DSU
			stick_to_dsu(&ls, &d[9], &d[10]);
			stick_to_dsu(&rs, &d[11], &d[12]);
			d[13] = (b1 & 0x80) ? 255 : 0;
			d[14] = (b1 & 0x40) ? 255 : 0;
			d[15] = (b1 & 0x20) ? 255 : 0;
			d[16] = (b1 & 0x10) ? 255 : 0;
			d[17] = (b2 & 0x80) ? 255 : 0;	// Y
			d[18] = (b2 & 0x40) ? 255 : 0;	// B
			d[19] = (b2 & 0x20) ? 255 : 0;	// A
			d[20] = (b2 & 0x10) ? 255 : 0;	// X
			d[21] = (b2 & 0x08) ? 255 : 0;
			d[22] = (b2 & 0x04) ? 255 : 0;
			d[23] = (b2 & 0x02) ? 255 : 0;
			d[24] = (b2 & 0x01) ? 255 : 0;
			// d[25..36] : points de contact du pave tactile, inutilises.
			put_u64(d + 37, t / 1000);	// horodatage en microsecondes
			put_f32(d + 45, ACCEL_X_SIGN * six.acceleration.x);
			put_f32(d + 49, ACCEL_Y_SIGN * six.acceleration.z);
			put_f32(d + 53, ACCEL_Z_SIGN * -six.acceleration.y);
			// La Switch donne des tours par seconde ; le protocole attend des degres par seconde.
			put_f32(d + 57, GYRO_PITCH_SIGN * six.angular_velocity.x * 360.0f);
			put_f32(d + 61, GYRO_YAW_SIGN * six.angular_velocity.z * 360.0f);
			put_f32(d + 65, GYRO_ROLL_SIGN * -six.angular_velocity.y * 360.0f);
			finish_packet(tx, len);

			for (int i = 0; i < MAX_CLIENTS; i++) {
				if (!clients[i].used)
					continue;
				if (t - clients[i].last_request > CLIENT_TIMEOUT_NS) {
					clients[i].used = false;
					continue;
				}
				if (sendto(sock, tx, len, 0, (struct sockaddr *)&clients[i].addr, sizeof(clients[i].addr)) == (ssize_t)len)
					stat_sent++;
				else
					stat_send_err++;
			}
		}

		// 5) Affichage.
		if (g_frame_ready) {
			if (!video_mode) {
				consoleExit(NULL);
				framebufferCreate(&fb, nwindowGetDefault(), SCREEN_W, SCREEN_H, PIXEL_FORMAT_RGBA_8888, 2);
				framebufferMakeLinear(&fb);
				video_mode = true;
			}
			if (mutexTryLock(&g_frame_lock)) {
				u32 stride;
				u8 *dst = (u8 *)framebufferBegin(&fb, &stride);
				for (int y = 0; y < SCREEN_H; y++)
					memcpy(dst + y * stride, g_frame + y * SCREEN_W * 4, SCREEN_W * 4);
				g_frame_ready = false;
				mutexUnlock(&g_frame_lock);
				framebufferEnd(&fb);
			}
		} else if (!video_mode && t - last_screen > 250000000ULL) {
			last_screen = t;
			int active = 0;
			for (int i = 0; i < MAX_CLIENTS; i++)
				if (clients[i].used) active++;
			printf("\x1b[16;1H--- Diagnostic ---\x1b[K\n");
			printf("DSU recu    : version %lu | ports %lu | donnees %lu\x1b[K\n",
				(unsigned long)stat_version, (unsigned long)stat_ports, (unsigned long)stat_data);
			printf("Clients DSU : %d | envoyes %lu | erreurs %lu\x1b[K\n",
				active, (unsigned long)stat_sent, (unsigned long)stat_send_err);
			printf("Compagnon PC: %s\x1b[K\n", g_video_client >= 0 ? "connecte" : "en attente");
			printf("Stick G     : x=%ld y=%ld\x1b[K\n", (long)ls.x, (long)ls.y);
			consoleUpdate(NULL);
		}

		svcSleepThread(4000000);	// environ 250 tours par seconde
	}

	g_running = false;
	if (vthread_ok) {
		int fd = g_video_client;
		if (fd >= 0)
			shutdown(fd, SHUT_RDWR);
		threadWaitForExit(&vthread);
		threadClose(&vthread);
	}
	if (sock >= 0)
		close(sock);
	socketExit();
	hidStopSixAxisSensor(h_handheld);
	hidStopSixAxisSensor(h_fullkey);
	hidStopSixAxisSensor(h_dual[0]);
	hidStopSixAxisSensor(h_dual[1]);
	appletSetAutoSleepDisabled(false);
	appletSetMediaPlaybackState(false);
	if (video_mode)
		framebufferClose(&fb);
	else
		consoleExit(NULL);
	free(g_frame);
	return 0;
}
