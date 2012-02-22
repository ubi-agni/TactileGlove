#include "seriallineconnector.h"

SerialLineConnector::SerialLineConnector()
{
    connected = full_frame = false;
    for (int i=0; i < PACKET_SIZE_BYTES; i++)
    {
        buf[i] = 0x0;
    }
    full_frames_counter = 0;
    keep_going = true;
    enable_sliding_average = true;
    index_sliding_average = 0;
    slot_frame = (unsigned short*) malloc (NO_GLOVE_ELEMENTS*sizeof (unsigned short));
    for (int i= 0; i < SLIDING_AVG; i++)
    {
        for (int j=0; j < NO_GLOVE_ELEMENTS; j++)
            full_frame_sensor_data[i][j] = 0;
    }
}

bool
SerialLineConnector::connect_device(const char *device)
{
    fd = open (device,O_RDWR | O_NOCTTY | O_NONBLOCK );
    if (fd < 0)
    {
        perror (device);
        return false;
    }
    tcgetattr(fd,&oldtio); /* save current port settings */

    bzero(&newtio, sizeof(newtio));

    newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
    newtio.c_cc[VMIN]     = PACKET_SIZE_BYTES;   /* blocking read until 5 chars received */

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd,TCSANOW,&newtio);
    connected = true;
    return (true);
}

bool
SerialLineConnector::disconnect_device()
{
    if (connected)
    {
        connected = full_frame = false;
        sleep(1);
        tcsetattr(fd,TCSANOW,&oldtio);
        close(fd);
        full_frames_counter = 0;
    }
    return true;
}

bool
SerialLineConnector::check_packet()
{
    if ((buf[0] < 0x3d) || (buf[0] > 0x73))
        return false;
    if (buf[1] != 0x01)
        return false;
    if (buf[4] != 0x0)
        return false;
    return true;
}

unsigned int
SerialLineConnector::next_index(unsigned int i)
{
    if (i < NO_GLOVE_ELEMENTS)
        return i+1;
    else
        return (0);
}

bool
SerialLineConnector::get_full_frame(unsigned short *full_frame)
{
    if (full_frame)
    {
        full_frame_sensor_data_mutex.lock();
        memcpy((void*)full_frame,(void*)full_frame_sensor_data,(size_t)54*sizeof (unsigned short));
        full_frame_sensor_data_mutex.unlock();
        return true;
    }
    return false;
}

void
SerialLineConnector::get_sensor_data(unsigned short *sensor_frame)
{
    sensor_data_mutex.lock();
    memcpy((void*)sensor_frame,(void*) sensor_data,(size_t) 54*sizeof (unsigned short));
    sensor_data_mutex.unlock();
}



void
SerialLineConnector::update_field()
{
    static unsigned int fields_in_a_row=0;
    static unsigned int last_packet_id = 0;

    unsigned int index;

    index = buf[0] - 0x3d;

    sensor_data_mutex.lock();
    sensor_data[index] = 0x0fff - (((0x0f&buf[2])*0x100) + (buf[3]));
    sensor_data_mutex.unlock();
    if (index == next_index(last_packet_id))
        fields_in_a_row++;
    else
    {
        fields_in_a_row = 0;
        fprintf (stderr,"index = 0x%x, next_index = 0x%x\n",index,next_index(last_packet_id));
    }
    last_packet_id = index;

    if (fields_in_a_row > 54)
    {
        full_frame = true;
    }
    else
        full_frame = false;
    if (full_frame && last_packet_id == (NO_GLOVE_ELEMENTS-1))
    {
        index_sliding_average++;
        if (index_sliding_average >= SLIDING_AVG)
            index_sliding_average = 0;
        full_frame_sensor_data_mutex.lock();
        sensor_data_mutex.lock();
        memcpy((void*)full_frame_sensor_data[index_sliding_average],(void*)sensor_data,(size_t) 54*sizeof(unsigned short));
        sensor_data_mutex.unlock();
        full_frame_sensor_data_mutex.unlock();
        full_frames_counter++;
        sensor_data_mutex.lock();
        if (enable_sliding_average)
        {
            for (int i=0; i < NO_GLOVE_ELEMENTS; i++)
            {
                slot_frame[i] = 0;
                for (int j=0; j < SLIDING_AVG; j++)
                {
                    slot_frame[i] += full_frame_sensor_data[j][i];
                }
                slot_frame[i] /= SLIDING_AVG;
            }
        }
        else
        {
          memcpy((void*)slot_frame,(void*)sensor_data,(size_t) 54*sizeof(unsigned short));
        }
        sensor_data_mutex.unlock();
        emit read_frame(slot_frame);
        switch (full_frames_counter % 8)
        {
        case 0: emit full_frame_update_message((QString ("Receiving ...")));
            break;
        case 1: emit full_frame_update_message((QString ("Receiving  ...")));
            break;
        case 2: emit full_frame_update_message((QString ("Receiving   ...")));
            break;
        case 3: emit full_frame_update_message((QString ("Receiving    ...")));
            break;
        case 4: emit full_frame_update_message((QString ("Receiving     ...")));
            break;
        case 5: emit full_frame_update_message((QString ("Receiving    ...")));
            break;
        case 6: emit full_frame_update_message((QString ("Receiving   ...")));
            break;
        case 7: emit full_frame_update_message((QString ("Receiving  ...")));
            break;
        default: emit full_frame_update_message((QString ("...")));
            break;
        }
    }

}

void
SerialLineConnector::endthread()
{
    keep_going = false;
}

void
SerialLineConnector::run()
{
    int res;
    fd_set fdset;
    struct timeval timeout;
    //fprintf (stderr,"Entered run");
    while (keep_going)
    {
        if (!connected)
        {
            sleep (1);
            continue;
        }
        //fprintf (stderr,".");
        FD_ZERO (&fdset);
        FD_SET (fd,&fdset);
        timeout.tv_sec = 0;
        timeout.tv_usec = 100;
        if (-1 == (res = select (fd+1,&fdset,NULL,NULL,&timeout)))
        {
            perror ("select");
            exit (EXIT_FAILURE);
        }
        if (res > 0)
        {
            res = read(fd,buf,5);
            if (5 == res)
            {
                //fprintf (stderr,"~");
                if (check_packet())
                {
                    //fprintf (stderr,"!");
                    update_field();
                }
            }
        }
    }
    if (connected)
        disconnect_device();
}

