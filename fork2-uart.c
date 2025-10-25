#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/wait.h>

int initUART(const char *portName)
{
    int fd = open(portName, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror("Erreur ouverture du port");
        exit(1);
    }

    struct termios SerialPortSettings;
    tcgetattr(fd, &SerialPortSettings);

    cfsetispeed(&SerialPortSettings, B9600);
    cfsetospeed(&SerialPortSettings, B9600);

    SerialPortSettings.c_cflag &= ~PARENB;
    SerialPortSettings.c_cflag &= ~CSTOPB;
    SerialPortSettings.c_cflag &= ~CSIZE;
    SerialPortSettings.c_cflag |= CS8;
    SerialPortSettings.c_cflag |= CREAD | CLOCAL;

    SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);
    SerialPortSettings.c_oflag &= ~OPOST;

    SerialPortSettings.c_cc[VMIN] = 1;  // attendre 1 caractère
    SerialPortSettings.c_cc[VTIME] = 0; // sans délai

    tcsetattr(fd, TCSANOW, &SerialPortSettings);

    return fd;
}

int main()
{
    char port[] = "/dev/ttyUSB0";
    int fd = initUART(port);

    pid_t pid1 = fork();

    if (pid1 == 0)
    {
        // Premier enfant → lecture
        printf("Processus Enfant 1 : lecture sur le port série...\n");
        char buffer[256];
        int n;
        while (1)
        {
            n = read(fd, buffer, sizeof(buffer) - 1);
            if (n > 0)
            {
                buffer[n] = '\0';
                printf("Lecture (%d octets) --> %s\n", n, buffer);
                if (buffer[0] == '!')
                {
                    printf("Fin Enfant 1 (lecture)\n");
                    break;
                }
            }
        }
        close(fd);
        exit(0);
    }

    pid_t pid2 = fork();

    if (pid2 == 0)
    {
        // Deuxième enfant → écriture
        printf("Processus Enfant 2 : écriture sur le port série...\n");
        char buffer[256];
        while (1)
        {
            printf("Entrer un message ('q' pour quitter) : ");
            fgets(buffer, sizeof(buffer), stdin);
            if (buffer[0] == 'q')
            {
                printf("Fin Enfant 2 (écriture)\n");
                break;
            }
            write(fd, buffer, strlen(buffer));
        }
        close(fd);
        exit(0);
    }

    // Processus parent → fait autre chose
    int n = 1;
    while (n < 10)
    {
        printf("Processus Principal : fait quelques trucs... (%d)\n", n);
        n++;
        sleep(3);
    }

    // Attendre la fin des enfants
    wait(NULL);
    wait(NULL);

    close(fd);
    printf("Fin du processus Principal\n");

    return 0;
}
