#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <termios.h>
#include <unistd.h>

namespace
{
constexpr const char *DEFAULT_UART_DEVICE = "/dev/ttyAMA3";
constexpr speed_t UART_BAUD = B115200;

bool configureUart(int fd)
{
  termios tty{};
  if (tcgetattr(fd, &tty) != 0)
  {
    std::cerr << "Erreur tcgetattr: " << std::strerror(errno) << "\n";
    return false;
  }

  cfsetispeed(&tty, UART_BAUD);
  cfsetospeed(&tty, UART_BAUD);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag &= ~(IGNBRK | IXON | IXOFF | IXANY);
  tty.c_lflag = 0;
  tty.c_oflag = 0;

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 10; // timeout lecture 1.0s

  if (tcsetattr(fd, TCSANOW, &tty) != 0)
  {
    std::cerr << "Erreur tcsetattr: " << std::strerror(errno) << "\n";
    return false;
  }

  tcflush(fd, TCIOFLUSH);
  return true;
}

void printUsage(const char *binaryName)
{
  std::cout << "Usage:\n"
            << "  " << binaryName << " <commande> [device_uart] [wait_ms]\n\n"
            << "Exemples de commandes vers Teensy:\n"
            << "  m               (maintien position)\n"
            << "  s               (stop maintien)\n"
            << "  p               (demande position)\n"
            << "  g 12.0,8.5      (go to X,Y en pouces)\n"
            << "  r 4.0,-2.0      (deplacement relatif dX,dY)\n"
            << "  f 20            (avance test 20 cm)\n"
            << "  o 90            (rotation 90 degres)\n\n"
            << "Device UART par defaut: " << DEFAULT_UART_DEVICE << "\n"
            << "wait_ms par defaut: 1000 ms\n"
            << "RPi: TX GPIO14 (pin 8) -> RX Teensy pin 0,\n"
            << "     RX GPIO15 (pin 10) <- TX Teensy pin 1,\n"
            << "     GND commun obligatoire.\n";
}
} // namespace

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    printUsage(argv[0]);
    return 1;
  }

  std::string command = argv[1];
  const char *device = (argc >= 3) ? argv[2] : DEFAULT_UART_DEVICE;
  int waitMs = (argc >= 4) ? std::stoi(argv[3]) : 1000;
  if (waitMs < 1)
  {
    waitMs = 1;
  }

  int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0)
  {
    std::cerr << "Impossible d'ouvrir " << device << ": " << std::strerror(errno) << "\n";
    return 2;
  }

  if (!configureUart(fd))
  {
    close(fd);
    return 3;
  }

  std::string payload = command + "\n";
  ssize_t written = write(fd, payload.c_str(), payload.size());
  if (written < 0)
  {
    std::cerr << "Erreur ecriture UART: " << std::strerror(errno) << "\n";
    close(fd);
    return 4;
  }

  std::string response;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(waitMs);

  while (std::chrono::steady_clock::now() < deadline)
  {
    char buffer[256]{};
    ssize_t received = read(fd, buffer, sizeof(buffer) - 1);
    if (received > 0)
    {
      response.append(buffer, buffer + received);
      continue;
    }

    if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
    {
      std::cerr << "Erreur lecture UART: " << std::strerror(errno) << "\n";
      close(fd);
      return 5;
    }
  }

  if (!response.empty())
  {
    std::cout << "Reponse Teensy: " << response;
  }
  else
  {
    std::cout << "Aucune reponse (timeout).\n";
  }

  close(fd);
  return 0;
}
