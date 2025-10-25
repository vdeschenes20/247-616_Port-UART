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

    SerialPortSettings.c_cc[VMIN] = 1;
    SerialPortSettings.c_cc[VTIME] = 0;

    tcsetattr(fd, TCSANOW, &SerialPortSettings);
    return fd;
}

int main()
{
    int fd = initUART("/dev/ttyUSB0");
    int pipeParentToChild[2];
    int pipeChildToParent[2];

    // Création des deux tuyaux
    pipe(pipeParentToChild);
    pipe(pipeChildToParent);

    pid_t pid1 = fork();

    if (pid1 == 0)
    {
        // Enfant 1 → lecture du port série et envoi au parent
        close(pipeChildToParent[0]); // on n'utilise pas la lecture du tuyau
        char buf[256];
        int n;
        while (1)
        {
            n = read(fd, buf, sizeof(buf) - 1);
            if (n > 0)
            {
                buf[n] = '\0';
                write(pipeChildToParent[1], buf, n);
                if (buf[0] == '!')
                {
                    printf("Fin Enfant 1 (lecture)\n");
                    break;
                }
            }
        }
        close(pipeChildToParent[1]);
        close(fd);
        exit(0);
    }

    pid_t pid2 = fork();

    if (pid2 == 0)
    {
        // Enfant 2 → lit le terminal et écrit sur le port série
        close(pipeParentToChild[1]); // on n'écrit pas dans ce tuyau
        char buf[256];
        while (1)
        {
            printf("Entrer un message ('q' pour quitter): ");
            fgets(buf, sizeof(buf), stdin);
            if (buf[0] == 'q')
            {
                printf("Fin Enfant 2 (écriture)\n");
                break;
            }
            write(fd, buf, strlen(buf));
        }
        close(pipeParentToChild[0]);
        close(fd);
        exit(0);
    }

    // Processus principal
    close(pipeChildToParent[1]); // ferme écriture
    close(pipeParentToChild[0]); // ferme lecture

    char buf[256];
    int n;
    int compteur = 1;
    while (compteur <= 10)
    {
        // Lit les messages venant de l’enfant 1
        n = read(pipeChildToParent[0], buf, sizeof(buf) - 1);
        if (n > 0)
        {
            buf[n] = '\0';
            printf("Parent a reçu du pipe: %s\n", buf);
        }
        printf("Parent fait autre chose... (%d)\n", compteur);
        sleep(2);
        compteur++;
    }

    wait(NULL);
    wait(NULL);
    close(fd);
    printf("Fin du processus Principal\n");
    return 0;
}
