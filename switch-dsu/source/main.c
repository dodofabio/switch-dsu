// Switch DSU : transforme la Switch en manette pour Cemu (protocole DSU / cemuhook).
// Envoie boutons, sticks, gyroscope, accéléromètre et tactile en UDP sur le port 26760.
// Dans Cemu : Paramètres d'entrée > API "DSUController" > IP de la Switch, port 26760.

#include <switch.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define DSU_PORT		26760
#define DSU_VERSION		1001
#define MAX_CLIENTS		4
#define CLIENT_TIMEOUT_NS	5000000000ULL	// un client qui ne redemande rien pendant 5 s est oublié

#define MSG_VERSION	0x100000
#define MSG_PORTS	0x100001
#define MSG_DATA	0x100002

// Sens des axes : passe une valeur à -1.0f si un mouvement est inversé dans Cemu.
#define GYRO_PITCH_SIGN	1.0f
#define GYRO_YAW_SIGN	1.0f
#define GYRO_ROLL_SIGN	1.0f
#define ACCEL_X_SIGN	1.0f
#define ACCEL_Y_SIGN	1.0f
#define ACCEL_Z_SIGN	1.0f

// Un client (Cemu) qui a demandé à recevoir les données.
typedef struct {
	bool			used;		// emplacement occupé ou non
	struct sockaddr_in	addr;		// adresse IP + port du PC
	u64			last_request;	// instant de la dernière demande (ns)
} Client;

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

// En-tête commun à tous les paquets envoyés par le serveur (16 octets).
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

// Début commun des réponses "ports" et "données" (11 octets) : décrit la manette du slot 0.
static void write_slot_info(u8 *p, u8 slot, bool connected)
{
	p[0] = slot;
	p[1] = connected ? 2 : 0;	// 2 = connectée
	p[2] = connected ? 2 : 0;	// 2 = gyroscope complet
	p[3] = connected ? 2 : 0;	// 2 = sans fil
	memset(p + 4, 0, 6);		// adresse MAC (non utilisée)
	if (connected)
		p[9] = 0xAA;		// fausse MAC non nulle pour que Cemu l'accepte
	p[10] = connected ? 0x05 : 0;	// batterie : pleine
}

static u8 stick_to_u8(s32 v, bool invert)
{
	s32 r = 128 + (invert ? -v : v) / 256;
	if (r < 0) r = 0;
	if (r > 255) r = 255;
	return (u8)r;
}

int main(int argc, char **argv)
{
	consoleInit(NULL);
	crc_init();

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

	// Empêche la mise en veille automatique pendant qu'on joue sur le PC.
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

	struct in_addr ip;
	ip.s_addr = (in_addr_t)gethostid();

	printf("\x1b[2J\x1b[1;1H");
	printf("=== Switch DSU - manette pour Cemu ===\n\n");
	if (sock < 0) {
		printf("ERREUR : impossible d'ouvrir le reseau.\n");
		printf("Verifie que la Switch est connectee au Wi-Fi.\n");
	} else {
		printf("Dans Cemu (Parametres d'entree) :\n");
		printf("  API      : DSUController\n");
		printf("  IP       : %s\n", inet_ntoa(ip));
		printf("  Port     : %d\n\n", DSU_PORT);
		printf("Le PC et la Switch doivent etre sur le meme reseau.\n");
	}
	printf("\nQuitter : appuyer sur + et - en meme temps.\n");

	Client clients[MAX_CLIENTS];
	memset(clients, 0, sizeof(clients));
	u32 server_id = (u32)now_ns() ^ 0x5D5A1234u;
	u32 packet_number = 0;
	u8 rx[128];
	u8 tx[128];

	// Compteurs de diagnostic affiches a l'ecran.
	u32 stat_version = 0, stat_ports = 0, stat_data = 0, stat_other = 0;
	u32 stat_sent = 0, stat_send_err = 0;
	char last_pc[32] = "aucun";
	u64 last_screen = 0;

	while (appletMainLoop()) {
		padUpdate(&pad);
		u64 held = padGetButtons(&pad);
		if ((held & HidNpadButton_Plus) && (held & HidNpadButton_Minus))
			break;

		u64 t = now_ns();

		// 1) Lecture des demandes de Cemu.
		if (sock >= 0) {
			for (;;) {
				struct sockaddr_in from;
				socklen_t fromlen = sizeof(from);
				ssize_t n = recvfrom(sock, rx, sizeof(rx), 0, (struct sockaddr *)&from, &fromlen);
				if (n < 20 || memcmp(rx, "DSUC", 4) != 0)
					break;
				u32 type;
				memcpy(&type, rx + 16, 4);
				snprintf(last_pc, sizeof(last_pc), "%s:%u", inet_ntoa(from.sin_addr), ntohs(from.sin_port));
				if (type == MSG_VERSION) stat_version++;
				else if (type == MSG_PORTS) stat_ports++;
				else if (type == MSG_DATA) stat_data++;
				else stat_other++;

				if (type == MSG_VERSION) {
					size_t len = 24;
					memset(tx, 0, sizeof(tx));
					write_header(tx, len, server_id);
					put_u32(tx + 16, MSG_VERSION);
					put_u16(tx + 20, DSU_VERSION);
					finish_packet(tx, len);
					sendto(sock, tx, len, 0, (struct sockaddr *)&from, fromlen);
				} else if (type == MSG_PORTS && n >= 24) {
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

		// 3) Envoi des données aux clients enregistrés.
		if (sock >= 0) {
			size_t len = 100;
			memset(tx, 0, sizeof(tx));
			write_header(tx, len, server_id);
			put_u32(tx + 16, MSG_DATA);
			write_slot_info(tx + 20, 0, true);
			u8 *d = tx + 31;
			d[0] = 1;			// manette connectée
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
			d[8] = touch.count > 0 ? 1 : 0;		// bouton tactile
			d[9]  = stick_to_u8(ls.x, false);
			d[10] = stick_to_u8(ls.y, false);
			d[11] = stick_to_u8(rs.x, false);
			d[12] = stick_to_u8(rs.y, false);
			// Versions analogiques (0 ou 255) : croix, boutons, gâchettes.
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
			// Premier point de contact tactile.
			if (touch.count > 0) {
				d[25] = 1;
				d[26] = 0;
				put_u16(d + 27, (u16)touch.touches[0].x);
				put_u16(d + 29, (u16)touch.touches[0].y);
			}
			// d[31..36] : second contact, inutilisé.
			put_u64(d + 37, t / 1000);	// horodatage en microsecondes
			put_f32(d + 45, ACCEL_X_SIGN * six.acceleration.x);
			put_f32(d + 49, ACCEL_Y_SIGN * six.acceleration.z);
			put_f32(d + 53, ACCEL_Z_SIGN * -six.acceleration.y);
			// La Switch donne des tours par seconde ; le protocole attend des degrés par seconde.
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

		// Affichage du diagnostic, 4 fois par seconde.
		if (t - last_screen > 250000000ULL) {
			last_screen = t;
			int active = 0;
			for (int i = 0; i < MAX_CLIENTS; i++)
				if (clients[i].used) active++;
			printf("\x1b[14;1H--- Diagnostic ---\x1b[K\n");
			printf("Recu du PC  : version %lu | ports %lu | donnees %lu | autre %lu\x1b[K\n",
				(unsigned long)stat_version, (unsigned long)stat_ports,
				(unsigned long)stat_data, (unsigned long)stat_other);
			printf("Dernier PC  : %s\x1b[K\n", last_pc);
			printf("Clients     : %d\x1b[K\n", active);
			printf("Envoyes     : %lu | erreurs d'envoi : %lu\x1b[K\n",
				(unsigned long)stat_sent, (unsigned long)stat_send_err);
			printf("Stick G     : x=%ld y=%ld\x1b[K\n", (long)ls.x, (long)ls.y);
		}

		consoleUpdate(NULL);
		svcSleepThread(4000000);	// environ 250 envois par seconde
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
	consoleExit(NULL);
	return 0;
}
