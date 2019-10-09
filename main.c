#include "stdafx.h"
#include "mp4.h"
#include <malloc.h>
#include <string.h>


#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include "rtp.h"

#define CLIENT_IP       "127.0.0.1"
#define CLIENT_PORT     8554
#define FPS             25


static int createUdpSocket()
{
    int fd;
    int on = 1;

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(fd < 0)
        return -1;

    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (const char*)&on, sizeof(on));

    return fd;
}

static int rtpSendH264Frame(int socket, char* ip, int16_t port,
                            struct RtpPacket* rtpPacket, uint8_t* frame, uint32_t frameSize)
{
    uint8_t naluType; 
    int sendBytes = 0;
    int ret;
    int j;

    naluType = frame[0];

    if (frameSize <= RTP_MAX_PKT_SIZE) 
    {
        /*
         *   0 1 2 3 4 5 6 7 8 9
         *  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
         *  |F|NRI|  Type   | a single NAL unit ... |
         *  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
         */
        memcpy(rtpPacket->payload, frame, frameSize);
        //printf("-----------------------------------------------\n");
        //for(j = 0; j < frameSize; j++)
	//	printf("%x ",rtpPacket->payload[j]);

        ret = rtpSendPacket(socket, ip, port, rtpPacket, frameSize);
        if(ret < 0)
            return -1;

        rtpPacket->rtpHeader.seq++;
        sendBytes += ret;
        if ((naluType & 0x1F) == 7 || (naluType & 0x1F) == 8) 
            goto out;
    }
    else 
    {
        /*
         *  0                   1                   2
         *  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3
         * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
         * | FU indicator  |   FU header   |   FU payload   ...  |
         * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
         */

        /*
         *     FU Indicator
         *    0 1 2 3 4 5 6 7
         *   +-+-+-+-+-+-+-+-+
         *   |F|NRI|  Type   |
         *   +---------------+
         */

        /*
         *      FU Header
         *    0 1 2 3 4 5 6 7
         *   +-+-+-+-+-+-+-+-+
         *   |S|E|R|  Type   |
         *   +---------------+
         */

        int pktNum = frameSize / RTP_MAX_PKT_SIZE;       
        int remainPktSize = frameSize % RTP_MAX_PKT_SIZE; 
        int i, pos = 1;


        for (i = 0; i < pktNum; i++)
        {
            rtpPacket->payload[0] = (naluType & 0x60) | 28;
            rtpPacket->payload[1] = naluType & 0x1F;
            
            if (i == 0) 
                rtpPacket->payload[1] |= 0x80; // start
            else if (remainPktSize == 0 && i == pktNum - 1) 
                rtpPacket->payload[1] |= 0x40; // end

            memcpy(rtpPacket->payload+2, frame+pos, RTP_MAX_PKT_SIZE);
            //printf("-----------------------------------------------\n");
            //for(j = 0; j < RTP_MAX_PKT_SIZE+2; j++)
		//printf("%x ",rtpPacket->payload[j]);

            ret = rtpSendPacket(socket, ip, port, rtpPacket, RTP_MAX_PKT_SIZE+2);
            if(ret < 0)
                return -1;

            rtpPacket->rtpHeader.seq++;
            sendBytes += ret;
            pos += RTP_MAX_PKT_SIZE;
        }


        if (remainPktSize > 0)
        {
            rtpPacket->payload[0] = (naluType & 0x60) | 28;
            rtpPacket->payload[1] = naluType & 0x1F;
            rtpPacket->payload[1] |= 0x40; //end

            memcpy(rtpPacket->payload+2, frame+pos, remainPktSize+2);
            //printf("-----------------------------------------------\n");
            //for(j = 0; j < remainPktSize+2; j++)
		//printf("%x ",rtpPacket->payload[j]);

            ret = rtpSendPacket(socket, ip, port, rtpPacket, remainPktSize+2);
            if(ret < 0)
                return -1;

            rtpPacket->rtpHeader.seq++;
            sendBytes += ret;
        }
    }

out:

    return sendBytes;
}

static int rtpSendAACFrame(int socket, char* ip, int16_t port,
                            struct RtpPacket* rtpPacket, uint8_t* frame, uint32_t frameSize)
{
    int ret;
    int j;
    rtpPacket->payload[0] = 0x00;
    rtpPacket->payload[1] = 0x10;
    rtpPacket->payload[2] = (frameSize & 0x1FE0) >> 5; //高8位
    rtpPacket->payload[3] = (frameSize & 0x1F) << 3; //低5位

    memcpy(rtpPacket->payload+4, frame, frameSize);


    //for(j = 0; j < frameSize+4; j++)
	//printf("%x ",rtpPacket->payload[j]);

    ret = rtpSendPacket(socket, ip, port, rtpPacket, frameSize+4);
    if(ret < 0)
    {
        printf("failed to send rtp packet\n");
        return -1;
    }

    rtpPacket->rtpHeader.seq++;

    /*
     * 如果采样频率是44100
     * 一般AAC每个1024个采样为一帧
     * 所以一秒就有 44100 / 1024 = 43帧
     * 时间增量就是 44100 / 43 = 1025
     * 一帧的时间为 1 / 43 = 23ms
     */
    rtpPacket->rtpHeader.timestamp += 1025;

    return 0;
}


int main(int argc, char* argv[])
{
   int i;
   int sampleCount;
   mp4_box_t *root = NULL;
   mp4_box_t *v_stsz = NULL, *v_stco = NULL, *v_stsc = NULL, *v_avc1 = NULL, *v_avcC = NULL;
   mp4_box_t *a_stsz = NULL, *a_stco = NULL, *a_stsc = NULL;
   stream_t* fd = NULL;

   unsigned long filesize = 0;
   BUFFER_t *buffer = NULL;
   FILE *file = fopen("test.mp4","rb");
   fseek(file,0L,SEEK_END);
   filesize = ftell(file);
   fseek(file,0L,SEEK_SET); 
   buffer = (BUFFER_t *)malloc(sizeof(BUFFER_t));
   buffer->begin_addr = (unsigned char *)malloc(filesize);
   buffer->buf = (unsigned char *)malloc(filesize);
   fread(buffer->begin_addr,filesize,1,file);
   memcpy(buffer->buf,buffer->begin_addr,filesize);
   (*buffer).offset = 0;
   (*buffer).filesize = filesize;
   fd = create_buffer_stream();
   if (buffer_open(fd, buffer) == 0)
      return -1;

   root = MP4_BoxGetRootFromBuffer(fd,filesize);

   /********************
    * video box info
    ********************/

   v_avc1 = MP4_BoxSearchBox(root,ATOM_avc1);
   printf("video box is %c%c%c%c  type %x \n"
		,v_avc1->i_type&0x000000ff
		,(v_avc1->i_type&0x0000ff00)>>8
		,(v_avc1->i_type&0x00ff0000)>>16
		,(v_avc1->i_type&0xff000000)>>24
		,v_avc1->i_type);

   v_avcC = MP4_BoxSearchBox(v_avc1->p_first,ATOM_avcC);
   printf("video box is %c%c%c%c  type %x \n"
		,v_avcC->i_type&0x000000ff
		,(v_avcC->i_type&0x0000ff00)>>8
		,(v_avcC->i_type&0x00ff0000)>>16
		,(v_avcC->i_type&0xff000000)>>24
		,v_avcC->i_type);

   v_stsz = MP4_BoxSearchBox(root,ATOM_stsz);
   printf("video box is %c%c%c%c  type %x  sample_count %d\n"
		,v_stsz->i_type&0x000000ff
		,(v_stsz->i_type&0x0000ff00)>>8
		,(v_stsz->i_type&0x00ff0000)>>16
		,(v_stsz->i_type&0xff000000)>>24
		,v_stsz->i_type
		,v_stsz->data.p_stsz->sample_count);
		
   int v_stsz_sample_count =  v_stsz->data.p_stsz->sample_count;
   int v_stsz_sample_size[v_stsz_sample_count];
   for(sampleCount = 0; sampleCount < v_stsz_sample_count; sampleCount++)
   {
	   v_stsz_sample_size[sampleCount] =  v_stsz->data.p_stsz->entry_size[sampleCount];
	   //printf("No %d smaple size %d\n",sampleCount+1,v_stsz_sample_size[sampleCount]);
   }
   v_stco = MP4_BoxSearchBox(root,ATOM_stco);
   printf("video box is %c%c%c%c  type %x  sample_count %d\n"
		,v_stco->i_type&0x000000ff
		,(v_stco->i_type&0x0000ff00)>>8
		,(v_stco->i_type&0x00ff0000)>>16
		,(v_stco->i_type&0xff000000)>>24
		,v_stco->i_type
		,v_stco->data.p_stco->sample_size);
		
   int v_stco_entry_count =  v_stco->data.p_stco->sample_size;
   int v_stco_chunk_offset[v_stco_entry_count];
   for(sampleCount = 0; sampleCount < v_stco_entry_count; sampleCount++) 
   {
	   v_stco_chunk_offset[sampleCount] =  v_stco->data.p_stco->entry_size[sampleCount*2];
  	   //printf("No %d Chunk offset %d\n",sampleCount+1,v_stco_chunk_offset[sampleCount]);
   }
   	
   v_stsc = MP4_BoxSearchBox(root,ATOM_stsc);
   printf("video box is %c%c%c%c  type %x  entry_count %d\n"
		,v_stsc->i_type&0x000000ff
		,(v_stsc->i_type&0x0000ff00)>>8
		,(v_stsc->i_type&0x00ff0000)>>16
		,(v_stsc->i_type&0xff000000)>>24
		,v_stsc->i_type
		,v_stsc->data.p_stsc->entry_count);
	
    int v_stsc_entry_count = v_stsc->data.p_stsc->entry_count;
    int v_stsc_first_chunk[v_stsc_entry_count];
    int v_stsc_samples_per_chunk[v_stsc_entry_count];
    for(sampleCount = 0; sampleCount < v_stsc_entry_count; sampleCount++) 
    {
	   v_stsc_first_chunk[sampleCount] = v_stsc->data.p_stsc->first_chunk[sampleCount];
	   v_stsc_samples_per_chunk[sampleCount] = v_stsc->data.p_stsc->samples_per_chunk[sampleCount];
  	   //printf("No %d Chunk offset %d\n",sampleCount+1,v_stco_chunk_offset[sampleCount]);
    }

   /********************
    * audio box info
    ********************/

   a_stsz = MP4_BoxSearchBox(v_stsz->p_next,ATOM_stsz);
   printf("audio box is %c%c%c%c  type %x  sample_count %d\n"
		,a_stsz->i_type&0x000000ff
		,(a_stsz->i_type&0x0000ff00)>>8
		,(a_stsz->i_type&0x00ff0000)>>16
		,(a_stsz->i_type&0xff000000)>>24
		,a_stsz->i_type
		,a_stsz->data.p_stsz->sample_count);
		
   int a_stsz_sample_count =  a_stsz->data.p_stsz->sample_count;
   int a_stsz_sample_size[a_stsz_sample_count];
   for(sampleCount = 0; sampleCount < a_stsz_sample_count; sampleCount++)
   {
	   a_stsz_sample_size[sampleCount] =  a_stsz->data.p_stsz->entry_size[sampleCount];
	   //printf("No %d smaple size %d\n",sampleCount+1,a_stsz_sample_size[sampleCount]);
   }

   a_stco = MP4_BoxSearchBox(v_stco->p_next,ATOM_stco);
   printf("audio box is %c%c%c%c  type %x  sample_count %d\n"
		,a_stco->i_type&0x000000ff
		,(a_stco->i_type&0x0000ff00)>>8
		,(a_stco->i_type&0x00ff0000)>>16
		,(a_stco->i_type&0xff000000)>>24
		,a_stco->i_type
		,a_stco->data.p_stco->sample_size);
		
   int a_stco_entry_count =  a_stco->data.p_stco->sample_size;
   int a_stco_chunk_offset[a_stco_entry_count];
   for(sampleCount = 0; sampleCount < a_stco_entry_count; sampleCount++) 
   {
	   a_stco_chunk_offset[sampleCount] =  a_stco->data.p_stco->entry_size[sampleCount*2];
  	   //printf("No %d Chunk offset %d\n",sampleCount+1,a_stco_chunk_offset[sampleCount]);
   }
   	
   a_stsc = MP4_BoxSearchBox(v_stsc->p_next,ATOM_stsc);
   printf("audio box is %c%c%c%c  type %x  entry_count %d\n"
		,a_stsc->i_type&0x000000ff
		,(a_stsc->i_type&0x0000ff00)>>8
		,(a_stsc->i_type&0x00ff0000)>>16
		,(a_stsc->i_type&0xff000000)>>24
		,a_stsc->i_type
		,a_stsc->data.p_stsc->entry_count);
	
   int a_stsc_entry_count = a_stsc->data.p_stsc->entry_count;
   int a_stsc_first_chunk[a_stsc_entry_count];
   int a_stsc_samples_per_chunk[a_stsc_entry_count];
   for(sampleCount = 0; sampleCount < a_stsc_entry_count; sampleCount++) 
   {
	   a_stsc_first_chunk[sampleCount] = a_stsc->data.p_stsc->first_chunk[sampleCount];
	   a_stsc_samples_per_chunk[sampleCount] = a_stsc->data.p_stsc->samples_per_chunk[sampleCount];
  	   //printf("No %d Chunk offset %d\n",sampleCount+1,a_stco_chunk_offset[sampleCount]);
   }
   	
   MP4_BoxFreeFromBuffer( root );
   buffer_close(fd);
   destory_buffer_stream(fd);

   /********************
    * check box info
    ********************/

//   for(i = 0; i < v_stsc_entry_count; i++)
//	printf("%d\n",v_stsc_first_chunk[i]);

//   for(i = 0; i < v_stsc_entry_count; i++)
//	printf("%d\n",v_stsc_samples_per_chunk[i]);

//   for(i = 0; i < v_stco_entry_count; i++)
//	printf("%d\n",v_stco_chunk_offset[i]);

//   for(i = 0; i < v_stsz_sample_count; i++)
//	printf("%d\n",v_stsz_sample_size[i]);
    
//   for(i = 0; i < a_stsc_entry_count; i++)
//	printf("%d\n",a_stsc_first_chunk[i]);

//   for(i = 0; i < a_stsc_entry_count; i++)
//	printf("%d\n",a_stsc_samples_per_chunk[i]);

//   for(i = 0; i < a_stco_entry_count; i++)
//	printf("%d\n",a_stco_chunk_offset[i]);

//   for(i = 0; i < a_stsz_sample_count; i++)
//	printf("%d\n",a_stsz_sample_size[i]);
 

   int buff_tmp[4] = {0, 0, 0, 0};
   int avcC_type[4] = {0x43, 0x63, 0x76, 0x61};
   int avcC_start_offset = 0;
   int sps_len_offset[2] = {0, 0};
   int sps_len = 0;
   int sps_start_offset = 0;
   int pps_len_offset[2] = {0, 0};
   int pps_len = 0;
   int pps_start_offset = 0;
   int offset = 0;
   uint8_t *frame_buff = (uint8_t*)malloc(300000);


   int socket;
   socket = createUdpSocket();

   int a_socket;
   a_socket = createUdpSocket();

   struct RtpPacket* rtpPacket;
   rtpPacket = (struct RtpPacket*)malloc(500000);
   rtpHeaderInit(rtpPacket, 0, 0, 0, RTP_VESION, RTP_PAYLOAD_TYPE_H264, 0, 0, 0, 0x88923423);

   struct RtpPacket* a_rtpPacket;
   a_rtpPacket = (struct RtpPacket*)malloc(500000);
   rtpHeaderInit(a_rtpPacket, 0, 0, 0, RTP_VESION, RTP_PAYLOAD_TYPE_AAC, 1, 0, 0, 0x32411);
   
   fd = create_file_stream();
   if (stream_open(fd, "test.mp4", MODE_READ) == 0)
      return -1;
   //video
  
   int v_stsc_no = 0;
   int v_stsz_no = 0;
   int v_chunk_no = 0;
   int v_stsc_first_chunk_num = 0;
   
   int a_stsc_no = 0;
   int a_stsz_no = 0;
   int a_chunk_no = 0;
   int a_stsc_first_chunk_num = 0;
   
   int av_switch = 0;
   while(1)
   {
	  if(offset < v_stco_chunk_offset[0])
	  {
		  // send sps by rtp
		  if(offset == sps_start_offset)
		  {
			  stream_read(fd, frame_buff, sps_len);
			  rtpSendH264Frame(socket, CLIENT_IP, CLIENT_PORT,rtpPacket, frame_buff, sps_len);
			  rtpPacket->rtpHeader.timestamp += 90000/FPS;
			  usleep(1000*1000/FPS);
			  offset+=sps_len;
		  }
		  // send pps by rtp
		  else if(offset == pps_start_offset)
		  {
			  stream_read(fd, frame_buff, pps_len);
			  rtpSendH264Frame(socket, CLIENT_IP, CLIENT_PORT,rtpPacket, frame_buff, pps_len);
			  rtpPacket->rtpHeader.timestamp += 90000/FPS;
			  usleep(1000*1000/FPS);
			  offset+=pps_len;
		  }
		  
		  stream_read(fd, frame_buff, 1);
		  buff_tmp[3] = buff_tmp[2];
		  buff_tmp[2] = buff_tmp[1];
		  buff_tmp[1] = buff_tmp[0];
		  buff_tmp[0] = frame_buff[0];

		  // get avcC offset
		  if(buff_tmp[0] == avcC_type[0] && buff_tmp[1] == avcC_type[1] && buff_tmp[2] == avcC_type[2] && buff_tmp[3] == avcC_type[3])
		  {
			  avcC_start_offset = offset;
			  sps_len_offset[0] = avcC_start_offset + 7;
			  sps_len_offset[1] = sps_len_offset[0] + 1;
			  sps_start_offset = sps_len_offset[1] + 1;
		  }
		  // get sps offset
		  if(offset == sps_len_offset[0] || offset == sps_len_offset[1])
		  {
			  if(offset == sps_len_offset[0])
			  {
				  sps_len = frame_buff[0] << 8;
			  } 
			  else 
			  {
				  sps_len += frame_buff[0];
				  pps_len_offset[0] = sps_start_offset + sps_len + 1;
				  pps_len_offset[1] = pps_len_offset[0] + 1;
				  pps_start_offset = pps_len_offset[1] + 1;
			  }
		  }
		
		  // get pps offset
		  if(offset == pps_len_offset[0] || offset == pps_len_offset[1]) 
		  {
			  if(offset == pps_len_offset[0])
			  {
				  pps_len = frame_buff[0] << 8;
			  } 
			  else 
			  {
				  pps_len += frame_buff[0];
			  }
		  }
	  	  offset++;
	  }


	  else
	  {
		//video
		if(av_switch == 0)
		{
			if(v_stsc_first_chunk_num == 0)
				v_stsc_first_chunk_num = v_stsc_first_chunk[v_stsc_no + 1] - v_stsc_first_chunk[v_stsc_no];
			if(v_chunk_no < v_stsc_samples_per_chunk[v_stsc_no])
			{
				//printf("video no %d size %d \n",v_stsz_no+1,v_stsz_sample_size[v_stsz_no]);
			 	stream_read(fd, frame_buff, v_stsz_sample_size[v_stsz_no]);
		 		rtpSendH264Frame(socket, CLIENT_IP, CLIENT_PORT, rtpPacket, frame_buff + 4, v_stsz_sample_size[v_stsz_no] - 4);
			 	rtpPacket->rtpHeader.timestamp += 90000/FPS;
				usleep(1000*1000/FPS);
				v_chunk_no++;
				v_stsz_no++;
			}
			else
			{
				v_chunk_no = 0;
				v_stsc_first_chunk_num--;
				av_switch++;
				if(v_stsc_first_chunk_num == 0)
				{
					if(v_stsc_no + 1 != v_stsc_entry_count)
						v_stsc_no++;				
				}
			}
		}

		//audio
		else if(av_switch == 1)
		{
			if(a_stsc_first_chunk_num == 0)
		        	a_stsc_first_chunk_num = a_stsc_first_chunk[a_stsc_no + 1] - a_stsc_first_chunk[a_stsc_no];
			if(a_chunk_no < a_stsc_samples_per_chunk[a_stsc_no])
			{
			 	stream_read(fd, frame_buff, a_stsz_sample_size[a_stsz_no]);
		 		rtpSendAACFrame(socket, CLIENT_IP, CLIENT_PORT, a_rtpPacket, frame_buff, a_stsz_sample_size[a_stsz_no]);
				//usleep(23000);
				a_chunk_no++;
				a_stsz_no++;
			}
			else
			{
				a_chunk_no = 0;
				a_stsc_first_chunk_num--;
				av_switch--;
				if(a_stsc_first_chunk_num == 0)
				{
					if(a_stsc_no + 1 != a_stsc_entry_count)
						a_stsc_no++;	
				}		

			}
		}

	}

		
	if (v_stsz_no == v_stsz_sample_count )
	{
		printf("------------------------ 881 ------------------\n");
		break;
	}

	
	
   }



   
   stream_close(fd);
   destory_file_stream(fd);

  
   return 0;
}

