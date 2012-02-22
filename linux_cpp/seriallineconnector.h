#ifndef SERIALLINECONNECTOR_H
#define SERIALLINECONNECTOR_H

#include <QObject>
#include <QThread>
#include <QMutex>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "glovesvgpainter.h"

#define BAUDRATE B115200
#define SLIDING_AVG 10
#define MODEMDEVICE "/dev/ttyACM0"
#define PACKET_SIZE_BYTES 5

class SerialLineConnector : public QThread
{
    Q_OBJECT
public:
    SerialLineConnector();
    bool connect_device (const char* device);
    bool disconnect_device ();
    void endthread();
    bool get_full_frame (unsigned short* full_frame);
    void get_sensor_data (unsigned short* sensor_frame);
signals:
    void read_frame(unsigned short*);
    void full_frame_update_message (QString q);
public slots:

protected:
    void run();
private:
    struct termios oldtio,newtio;
    unsigned char buf[PACKET_SIZE_BYTES];
    int fd;
    bool connected;
    bool keep_going;
    unsigned short sensor_data[NO_GLOVE_ELEMENTS];
    unsigned short full_frame_sensor_data[SLIDING_AVG][NO_GLOVE_ELEMENTS];
    unsigned short* slot_frame;
    unsigned long int full_frames_counter;
    bool check_packet();
    void update_field();
    unsigned int next_index (unsigned int i);
    bool full_frame;
    bool enable_sliding_average;
    unsigned int index_sliding_average;
    QMutex sensor_data_mutex;
    QMutex full_frame_sensor_data_mutex;
};

#endif // SERIALLINECONNECTOR_H
