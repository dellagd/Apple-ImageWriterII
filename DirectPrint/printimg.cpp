#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

/*
 * In fast draft mode, set line spacing to 16/144
 * The resulting dot ratio is height:width of 1.38:1
 */ 

// Function credit: wallyk & RicoRico
int set_interface_attribs (int fd, int speed, int parity) {
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        perror("error from tcgetattr");
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        perror("error from tcsetattr");
        return -1;
    }
    return 0;
}

// Function credit: wallyk & RicoRico
void set_blocking (int fd, int should_block) {
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        perror("error from tggetattr");
        return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
        perror("error %d setting term attributes");
}

int init_port(char *portname){
	int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
        perror("error opening file");
        return 1;
	}

	set_interface_attribs (fd, B9600, 0);  // set speed to 9,600 bps, 8n1 (no parity)
	set_blocking (fd, 0);                // set no blocking
    
    return fd;
}

void send_line(int iw, char *line, size_t len){
    write(iw, strcat(line, "\n\r"), len+2);
}

void print_buffer(int iw, char *buf, size_t len){
    char seg[250];

    int i = 0;
    while (i < len){
        int amnt = 250;
        if (len - i < 250) amnt = len - i;
        memcpy(seg, (buf+i), amnt);
        write(iw, seg, amnt);
        i += 250;
        //printf("Done segment!\n");
        usleep(990000); //Don't overfill buffer
    }
    
    write(iw, "\n\r", 2);
}

char * read_text(char * filename){
    char * filebuf = (char*)calloc(sizeof(char),4000);
    char line[92];
    printf("Reading file %s...\n", filename);
    FILE *fp = fopen(filename, "r"); // do not use "rb"
    
    size_t i = 0;
    while (fgets(line, sizeof(line)-2, fp)) {
        //printf(line);
        if (strlen(line) >= 89) strcat(line, "\n");
        strcat(filebuf, line);
        printf(filebuf);
    }

    fclose(fp);
    printf("Done! closing file read\n");
    return filebuf;
}

void init_text(int iw){
    char std_sp[4] = {27, 'A'};
    write(iw, std_sp, 4);
}

void init_graphics(int iw, int bytes){
    if (bytes > 1500) printf("Too many bytes for one section!");
    
    char gr_sp[4] = {27, 'T', '1', '6'};
    write(iw, gr_sp, 4);
    
    char num[5];
    sprintf(num, "%04d", bytes);
    char cmd[6] = {27, 71, 0, 0, 0, 0};
    memcpy(cmd+2, num, 4);
    write(iw, cmd, 6);
}

int main(int argc, char *argv[]){
	//if (argc != 2) printf("Please execute with filename of text file\n");
    
    char txt[40];
    char *portname = "/dev/ttyS0";
    int iw = init_port(portname);

    int linelength = 128;
    for(int foo = 0; foo < 16; foo++){
        init_graphics(iw, linelength);
        char s[1] = {0};

        for (size_t i = 0; i < linelength; i++) {
            s[0] = (0xFF&i);
            write(iw, s, 1);
        }
        
        write(iw, strcpy(txt, "\n"), 1);
    }
    
    init_text(iw);
    write(iw, "Test!\n\r", 7);
    write(iw, "Test!\n\r", 7);


    usleep ((7 + 25) * 10000);             // sleep enough to transmit the 7 plus
                                         // receive 25:  approx 100 uS per char transmit
	//char buf [100];
	//int n = read (fd, buf, sizeof buf);  // read up to 100 characters if ready to read

    return 0;
}
