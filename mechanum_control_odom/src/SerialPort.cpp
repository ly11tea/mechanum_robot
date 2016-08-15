#include "SerialPort.h"
#include <stdio.h>
#include <string>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <cstdlib>
#include <iostream>
#include <stdint.h>

SerialPort::SerialPort()
{
	portName = NULL;
	portBaudRate = B0;
	portIsOpen = false;
}

SerialPort::SerialPort(char *port, int baudRate)
{
	portName = port;
	portBaudRate = baudRate;
	portIsOpen = openPort();	
}

SerialPort::~SerialPort()
{
	close(fileDescriptor);
}

bool SerialPort::openPort()
{	
	printf("Trying to open port!\n");
	if(!portIsOpen && (portName != NULL) )
	{	
		fileDescriptor = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);	
	}
	printf("Finished  opening port!\n");
	if(fileDescriptor == -1)
	{
		perror("openPort(): Unable to open serial port");
		return false;
	}
	else
	{	printf("fcntl start \n");
		fcntl(fileDescriptor, F_SETFL, 0);
		printf("fcntl finished\n");
		portIsOpen = true;
		printf("configurePort start \n");
		configurePort(fileDescriptor, portBaudRate);
		printf("configurePort finished \n");
		return true;
	}
		
}

void SerialPort::closePort()
{
	if(portIsOpen) 
	{
		close(fileDescriptor);
		portName = NULL;
		fileDescriptor = -1;
	}
}

int SerialPort::readData( char *readBuffer, int bytesToRead)
{
	int bytesRead = -1;

	if(portIsOpen)
	{
		bytesRead = read(fileDescriptor, readBuffer, bytesToRead);
	}

	return bytesRead;
}

int SerialPort::writeData( char *writeBuffer, int bytesToWrite)
{
	int bytesWritten = -1;

	if(portIsOpen)
	{
		bytesWritten = write(fileDescriptor, writeBuffer, bytesToWrite);
	}
	
	return bytesWritten;
}
void SerialPort::setBaudRate(int baudRate)
{
	portBaudRate = baudRate;
}

void SerialPort::setPort(char *port)
{
	portName = port;
}

int SerialPort::configurePort(int fd, int baudRate)      // configure the port
{

	//printf("Opening 485 Bus Port: %s...",BUSPORT);fflush(stdout);

    // BAUD RATE ETC
	struct termios settings;


   tcgetattr( fileDescriptor, &settings  ); //Get the current settings for the port...
	//cfsetispeed(&settings, B460800);	//Set the baud rates to 19200...
	//cfsetospeed(&settings, B460800);
	settings.c_cflag |= (CLOCAL | CREAD); //Enable the receiver and set local mode...
	tcsetattr(fileDescriptor, TCSANOW, &settings); //Set the new settings for the port...
	
	//No parity (8N1):
	settings.c_cflag &= ~PARENB;
	settings.c_cflag &= ~CSTOPB;
	settings.c_cflag &= ~CSIZE;
	settings.c_cflag |= CS8;

	//Disable hardware flow control:
	//settings.c_cflag &= ~CNEW_RTSCTS;

	//Disable software flow control
	settings.c_iflag &= ~(IXON | IXOFF | IXANY);

	//Choosing Raw Input
	settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	settings.c_iflag &= ~INLCR;
	settings.c_iflag &= ~ICRNL;
	settings.c_iflag &= ~IGNBRK;
	settings.c_oflag &= ~ONLCR;
	settings.c_oflag &= ~OCRNL;
	settings.c_cc[VMIN] = 0;
	settings.c_cc[VTIME] = 0;
	settings.c_cflag = CS8 | CLOCAL | CREAD;


	//pic_port = open(BUSPORT, O_RDWR | O_NOCTTY | O_NDELAY);
	fileDescriptor = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fileDescriptor < 0)
	{
		printf("Error opening Sonar R485.\r\n");	fflush(stdout);
		return -1;
    }
	else
    {
    	printf("Opened Sonar RS485!.\r\n");	fflush(stdout);
    }

	//pic_port = open(BUSPORT, O_RDWR | O_NONBLOCK);
	cfsetospeed(&settings,B115200);            // 115200 baud
	cfsetispeed(&settings,B115200);            // 115200 baud
	tcsetattr(fileDescriptor, TCSANOW, &settings); 	//Set the new settings for the port...

	tcflush(fileDescriptor, TCIFLUSH);


		printf("Done.\r\n");
		
		return fileDescriptor;

}	

bool SerialPort::isOpen()
{
	return portIsOpen;
}

int SerialPort::writeVector( std::vector<uint8_t> const& bytes )
{
	
	 char writeBuffer[100] = {0};
	int bytesWritten  = -1;
         char *p_writeBuffer = writeBuffer;
	if(portIsOpen)
	{
		int bytesToWrite = bytes.size();
		int i=0;
		//printf("written data:");
		for (;i<bytesToWrite;i++)
			{writeBuffer[i] = (char) bytes[i];
			 //  printf("%d ",(unsigned char)writeBuffer[i] );
			}
			//printf("\n");
		//writeBuffer[i] = '\0';
		bytesWritten = writeData(p_writeBuffer, bytesToWrite);
		
		}
		//printf("byteswritten is %d\n",bytesWritten);
	return bytesWritten;
		
	/*  if(fileDescriptor == -1)
  { std::cerr << "Cannot write to closed serial port" << std::endl; return -1; }

  int bytesWritten = ::write(fileDescriptor, &bytes[0], bytes.size());
  if(bytesWritten != bytes.size())
    std::cerr << "Error writing to serial port: " << strerror(errno) << std::endl;
  return bytesWritten;*/
}

std::vector<uint8_t> SerialPort::readVector( size_t bytes )
{
	 if(!portIsOpen)
  { printf("Cannot read from closed serial port\n") ; return std::vector<uint8_t>(); }
    char arrayRead[1024] = {};
    char *p_arrayRead = arrayRead;
  std::vector<uint8_t> ret(bytes);
  int read_length = readData(p_arrayRead, bytes);
  ret.resize(read_length);
	for(int i=0;i<read_length;i++)
	   ret[i] = (uint8_t) arrayRead[i];
	return ret;
}

