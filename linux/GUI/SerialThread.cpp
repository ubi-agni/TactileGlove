#include "SerialThread.h"

SerialThread::SerialThread()
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
    slot_frame = (unsigned short*) malloc (NO_TAXELS*sizeof (unsigned short));
    for (int i= 0; i < SLIDING_AVG; i++)
    {
        for (int j=0; j < NO_TAXELS; j++)
            full_frame_sensor_data[i][j] = 0;
    }
}

bool
SerialThread::connect_device(const char *device)
{
    if ((fd = open (device, O_RDONLY | O_NOCTTY)) < 0) {
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
SerialThread::disconnect_device()
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
SerialThread::check_packet()
{
    if ((buf[0] < 0x3c) || (buf[0] > 0x7B))
        return false;
    if (buf[1] != 0x01)
        return false;
    if (buf[4] != 0x0)
        return false;
    return true;
}

void
SerialThread::recover()
{
  unsigned int start=0;
  fprintf(stderr,"recover from %2X %2X %2X %2X %2X\n",buf[0],buf[1],buf[2],buf[3],buf[4]);
  for (; start < PACKET_SIZE_BYTES; ++start) {
    if (buf[(start+1) % PACKET_SIZE_BYTES] == 0x01 && 
	buf[(start+4) % PACKET_SIZE_BYTES] == 0x00 && 
    buf[start] >= 0x3c && buf[start] <= 0x7B)
      break;
  }
  if (start < PACKET_SIZE_BYTES) {
    memmove(buf, buf+start, PACKET_SIZE_BYTES-start);
    ssize_t unused = read(fd,buf+PACKET_SIZE_BYTES-start,start);
    if (!check_packet() || (unused < 0)) fprintf(stderr, "this should not happen!\n");
  } else {
    fprintf (stderr, "could not recover\n");
  }
}

unsigned int
SerialThread::next_index( int i)
{
    if (++i < NO_TAXELS)
        return i;
    else
        return (0);
}

bool
SerialThread::get_full_frame(unsigned short *full_frame)
{
    if (full_frame)
    {
        full_frame_sensor_data_mutex.lock();
        memcpy((void*)full_frame,(void*)full_frame_sensor_data,(size_t)NO_TAXELS*sizeof (unsigned short));
        full_frame_sensor_data_mutex.unlock();
        return true;
    }
    return false;
}

void
SerialThread::get_sensor_data(unsigned short *sensor_frame)
{
    sensor_data_mutex.lock();
    memcpy((void*)sensor_frame,(void*) sensor_data,(size_t) NO_TAXELS*sizeof (unsigned short));
    sensor_data_mutex.unlock();
}



void
SerialThread::update_field()
{
    static unsigned int fields_in_a_row=0;
    static int last_packet_id = -1;

    unsigned int index;

    index = buf[0] - 0x3c;
    //fprintf (stderr,"index = 0x%x %u %u\n",index, fields_in_a_row,NO_GLOVE_ELEMENTS);
    if (index + 1 > NO_TAXELS) // We skip elements 74-7B
        return;
    sensor_data_mutex.lock();
    sensor_data[index] = 0x0fff - (((0x0f&buf[2])*0x100) + (buf[3]));
    sensor_data_mutex.unlock();
    if (index == next_index(last_packet_id))
        fields_in_a_row++;
    else
    {
        fields_in_a_row = 0;
        fprintf (stderr,"index = 0x%x, expected = 0x%x\n",index,next_index(last_packet_id));
    }
    last_packet_id = index;

    if (fields_in_a_row+1 >= NO_TAXELS)
    {
        full_frame = true;
    }
    else
        full_frame = false;
    if (full_frame && (last_packet_id == (NO_TAXELS-1)))
    {
      //fprintf (stderr,"Fullframe \n");
        index_sliding_average++;
        if (index_sliding_average >= SLIDING_AVG)
            index_sliding_average = 0;
        full_frame_sensor_data_mutex.lock();
        sensor_data_mutex.lock();
        memcpy((void*)full_frame_sensor_data[index_sliding_average],(void*)sensor_data,(size_t) NO_TAXELS*sizeof(unsigned short));
        sensor_data_mutex.unlock();
        full_frame_sensor_data_mutex.unlock();
        full_frames_counter++;
        sensor_data_mutex.lock();
        if (enable_sliding_average)
        {
            for (int i=0; i < NO_TAXELS; i++)
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
          memcpy((void*)slot_frame,(void*)sensor_data,(size_t) NO_TAXELS*sizeof(unsigned short));
        }
        sensor_data_mutex.unlock();
		  emit read_frame(slot_frame);
        switch ((full_frames_counter/2) % 8)
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
SerialThread::enable_send()
{
    send_update = true;
}

void
SerialThread::endthread()
{
    keep_going = false;
}

void SerialThread::run()
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
        timeout.tv_usec = 1000;
        if (-1 == (res = select (fd+1,&fdset,NULL,NULL,&timeout)))
        {
            perror ("select");
            exit (EXIT_FAILURE);
        }
        if (res > 0)
        {
            res = read(fd,buf,5);
            //printf("%d: %X %X %X %X %X\n",res, buf[0],buf[1],buf[2],buf[3],buf[4]);

            if (5 == res)
            {
                if (check_packet())
                {
                    update_field();
                } else {
                    recover();
                }
            }
        }
        else {fprintf (stderr, "!");}
    }
    if (connected)
        disconnect_device();
}

