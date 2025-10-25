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

    SerialPortSettings.c_cflag &= ~PARENB; // pas de parité
    SerialPortSettings.c_cflag &= ~CSTOPB; // 1 bit stop
    SerialPortSettings.c_cflag &= ~CSIZE;
    SerialPortSettings.c_cflag |= CS8;     // 8 bits
    SerialPortSettings.c_cflag |= CREAD | CLOCAL;

    SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // non-canonical
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);
    SerialPortSettings.c_oflag &= ~OPOST;

    SerialPortSettings.c_cc[VMIN] = 2;  // attendre au moins 1 caractère
    SerialPortSettings.c_cc[VTIME] = 0; // pas de timeout

    tcsetattr(fd, TCSANOW, &SerialPortSettings);

    return fd;
}

int main()
{
    char port[] = "/dev/ttyUSB0";
    int fd = initUART(port);

    pid_t pid = fork();

    if (pid < 0)
    {
        perror("Erreur fork");
        exit(1);
    }
    else if (pid == 0)
    {
        // Processus enfant → écriture
        printf("Je suis le processus Fils, j'écris sur le port série ce que j'entends sur la console...\n");
        char buffer[256];
        while (1)
        {
            printf("Entrer un message ('q' pour quitter) : ");
            fgets(buffer, sizeof(buffer), stdin);

            if (buffer[0] == 'q')
            {
                printf("Fin du Fils\n");
                break;
            }

            write(fd, buffer, strlen(buffer));
        }
        close(fd);
        exit(0);
    }
    else
    {
        // Processus parent → lecture
        printf("Je suis le processus Père, j'écris sur la console ce que j'entends sur le port série...\n");
        char readbuf[256];
        int n;

        while (1)
        {
            n = read(fd, readbuf, sizeof(readbuf) - 1);
            if (n > 0)
            {
                readbuf[n] = '\0';
                printf("processus Père: nombre d'octets reçus : %d --> %s\n", n, readbuf);
                if (readbuf[0] == '!')
                {
                    printf("Fin du Père\n");
                    break;
                }
            }
        }

        wait(NULL);
        close(fd);
    }

    return 0;
}
