/*
***************************************************************************
*
* Author: Teunis van Beelen
*
* Copyright (C) 2015 - 2023 Teunis van Beelen
*
* Email: teuniz@protonmail.com
*
***************************************************************************
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
***************************************************************************
*/


#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>

#include "tmc_dev.h"
#include "utils.h"



#define MAX_CMD_LEN     (255)
#define MAX_TRANSFER_LEN    (1024 * 1024 * 2)
#define USBTMC_TIMEOUT 5000


// to get bulk in/out endpoint
//   $ lsusb -v -d 1ab1:0517
#define EP_OUT 0x03
#define EP_IN  0x82



struct tmcdev * tmcdev_open(const char *device)
{
  struct tmcdev *dev;

  (void)device;


  dev = (struct tmcdev *)calloc(1, sizeof(struct tmcdev));
  if(dev == NULL)
  {
    return NULL;
  }

  dev->hdrbuf = (char *)calloc(1, MAX_TRANSFER_LEN + 1024);
  if(dev->hdrbuf == NULL)
  {
    free(dev);

    return NULL;
  }

  dev->buf = dev->hdrbuf;

  libusb_init(NULL);
  // hardcode DS1102Z-E
  dev->hndl = libusb_open_device_with_vid_pid(NULL, 0x1ab1, 0x0517);

  if(dev->hndl == NULL)
  {
    free(dev->hdrbuf);

    free(dev);

    return NULL;
  }

  dev->dev = libusb_get_device(dev->hndl);
  libusb_set_auto_detach_kernel_driver(dev->hndl, 1);
  libusb_claim_interface(dev->hndl, 0x00);

  return dev;
}


void tmcdev_close(struct tmcdev *dev)
{
  if(dev == NULL)
  {
    return;
  }

  libusb_release_interface(dev->hndl, 0x00);
  libusb_close(dev->hndl);
  libusb_exit(NULL);

  free(dev->hdrbuf);

  free(dev);
}

// convenience struct for usbtmc, modified from tinyusb
typedef enum {
  USBTMC_MSGID_DEV_DEP_MSG_OUT = 1u,
  USBTMC_MSGID_DEV_DEP_MSG_IN = 2u,
} usbtmc_msgid_enum;

typedef struct
{
  uint8_t MsgID;
  uint8_t bTag;
  uint8_t bTagInverse;
  uint8_t _reserved;
} usbtmc_msg_header_t;

typedef struct
{
  usbtmc_msg_header_t header;
  uint32_t TransferSize;
  uint8_t  EOM;
  uint8_t _reserved[3];
} usbtmc_msg_request_dev_dep_out;

typedef struct
{
  usbtmc_msg_header_t header;
  uint32_t TransferSize;
  uint8_t TermCharEnabled;
  uint8_t TermChar;
  uint8_t _reserved[2];
} usbtmc_msg_request_dev_dep_in;

typedef struct
{
  usbtmc_msg_header_t header;
  uint32_t TransferSize;
  uint8_t bmTransferAttributes;
  uint8_t _reserved[3];
} usbtmc_msg_dev_dep_msg_in_header_t;

static int usbtmc_write(struct tmcdev *dev, char *buf, size_t n)
{
  /* assume message length always smaller than max transfer len */
  uint8_t buf_raw[MAX_TRANSFER_LEN + sizeof(usbtmc_msg_request_dev_dep_out)+3];

  // assume little endian
  dev->btag = (dev->btag % 255) + 1;
  usbtmc_msg_request_dev_dep_out out_hdr = {
    .header = {
      .MsgID = USBTMC_MSGID_DEV_DEP_MSG_OUT,
      .bTag = dev->btag,
      .bTagInverse = ~dev->btag & 0xFF,
      ._reserved = 0x00
    },
    .TransferSize = n,
    .EOM = 0x01,
    ._reserved = {0x00}
  };

  int m = 0;

  memcpy(buf_raw, &out_hdr, sizeof(usbtmc_msg_request_dev_dep_out));
  m += sizeof(usbtmc_msg_request_dev_dep_out);
  memcpy(buf_raw+m, buf, n);
  m += n;

  size_t pad = (4-(n % 4)%4);
  while (pad-- > 0)
    buf_raw[m++] = 0;

  int rc;
  int actual_len;

  rc = libusb_bulk_transfer(dev->hndl, EP_OUT, buf_raw, m, &actual_len, USBTMC_TIMEOUT);
  if (rc < 0 || actual_len != m)
    return -1;

  return n;
}

static int usbtmc_read(struct tmcdev *dev, char *buf, size_t n, int initial)
{
  /* assume message length always smaller than max transfer len */
  uint8_t bufrecv_raw[MAX_TRANSFER_LEN + sizeof(usbtmc_msg_dev_dep_msg_in_header_t)+3];
  int rc;
  int actual_len;

  // send usbtmc bulk in header
  size_t read_len = MAX_TRANSFER_LEN;
  if (n < read_len)
    read_len = n;

  if (initial) {
    // assume little endian
    dev->btag = (dev->btag % 255) + 1;
    usbtmc_msg_request_dev_dep_in in_hdr = {
      .header = {
        .MsgID = USBTMC_MSGID_DEV_DEP_MSG_IN,
        .bTag = dev->btag,
        .bTagInverse = ~dev->btag & 0xFF,
        ._reserved = 0x00
      },
      .TransferSize = read_len,
      .TermCharEnabled = 0x00,
      .TermChar = 0x00,
      ._reserved = {0x00}
    };

    int m = sizeof(usbtmc_msg_request_dev_dep_in);
    rc = libusb_bulk_transfer(dev->hndl, EP_OUT, (void*)&in_hdr, m, &actual_len, USBTMC_TIMEOUT);
    if (rc < 0 || actual_len != m)
      return -1;
  }

  // read data
  int m = read_len+sizeof(usbtmc_msg_dev_dep_msg_in_header_t)+3;
  rc = libusb_bulk_transfer(dev->hndl, EP_IN, bufrecv_raw, m, &actual_len, USBTMC_TIMEOUT);
  if (rc < 0 || actual_len < 0)
    return -1;

  if (initial) {
    usbtmc_msg_dev_dep_msg_in_header_t resp_hdr;
    memcpy(&resp_hdr, bufrecv_raw, sizeof(resp_hdr));
    size_t transfer_size = resp_hdr.TransferSize;
    if (bufrecv_raw[sizeof(resp_hdr)] == '#') {
      transfer_size = actual_len-sizeof(resp_hdr);
      if (transfer_size > resp_hdr.TransferSize)
        transfer_size = resp_hdr.TransferSize;
    }
    memcpy(buf, bufrecv_raw+sizeof(resp_hdr), transfer_size);
    return transfer_size;
  }

  memcpy(buf, bufrecv_raw, actual_len);
  return actual_len;
}

int tmcdev_write(struct tmcdev *dev, const char *cmd)
{
  int i, n, len, qry=0;

  char buf[MAX_CMD_LEN + 16],
       str[256];

  if(dev == NULL)
  {
    return -1;
  }

  len = strlen(cmd);

  if(len > MAX_CMD_LEN)
  {
    printf("tmcdev error: command too long\n");

    return -1;
  }

  if(len < 2)
  {
    printf("tmcdev error: command too short\n");

    return -1;
  }

  if(cmd[len - 1] == '?')
  {
    qry = 1;
  }

  strlcpy(buf, cmd, MAX_CMD_LEN + 16);

  strlcat(buf, "\n", MAX_CMD_LEN + 16);

  if(!(!strncmp(buf, ":TRIG:STAT?", 11) ||  /* don't print these commands to the console */
       !strncmp(buf, ":TRIG:SWE?", 10) ||   /* because they are used repeatedly */
       !strncmp(buf, ":WAV:DATA?", 10) ||
       !strncmp(buf, ":WAV:MODE NORM", 14) ||
       !strncmp(buf, ":WAV:FORM BYTE", 14) ||
       !strncmp(buf, ":WAV:SOUR CHAN", 14) ||
       !strncmp(buf, ":TRIG:SWE?", 10) ||
       !strncmp(buf, ":ACQ:SRAT?", 10) ||
       !strncmp(buf, ":ACQ:MDEP?", 10) ||
       !strncmp(buf, ":MEAS:COUN:VAL?", 15) ||
       !strncmp(buf, ":FUNC:WREC:OPER?", 16) ||
       !strncmp(buf, ":FUNC:WREP:OPER?", 16) ||
       !strncmp(buf, ":FUNC:WREP:FMAX?", 16) ||
       !strncmp(buf, ":FUNC:WREC:FMAX?", 16) ||
       !strncmp(buf, ":FUNC:WREP:FCUR?", 16) ||
       !strncmp(buf, ":WAV:XOR?", 9)))
  {
    printf("tmc_dev write: %s", buf);
  }

  if((!strncmp(buf, "*RST", 4)) || (!strncmp(buf, ":AUT", 4)))
  {
    qry = 1;
  }

  n = usbtmc_write(dev, buf, strlen(buf));

  if(n != (len + 1))
  {
    printf("tmcdev error: device write error");

    return -1;
  }

  if(!qry)
  {
    for(i=0; i<20; i++)
    {
      usleep(25000);

      n = usbtmc_write(dev, "*OPC?\n", 6);

      if(n < 0)
      {
        printf("tmcdev error: device write error");

        return -1;
      }

      n = usbtmc_read(dev, str, 128, 1);

      if(n < 0)
      {
        printf("tmcdev error: device read error");

        return -1;
      }

      if(n == 2)
      {
        if(str[0] == '1')
        {
          break;
        }
      }
    }
  }

  return len;
}


/*
 * TMC Blockheader ::= #NXXXXXX: is used to describe
 * the length of the data stream, wherein, # is the start denoter of
 * the data stream; N is less than or equal to 9; the N figures
 * followed N represents the length of the data stream in bytes.
 * For example, #9001152054. Wherein, N is 9 and 001152054
 * represents that the data stream contains 1152054 bytes
 * effective data.
 * Reading from the file descriptor blocks,
 * there is a timeout of 5000 milli-Sec.
 */
int tmcdev_read(struct tmcdev *dev)
{
  int size, size2, len;

  char blockhdr[32];

  if(dev == NULL)
  {
    return -1;
  }

  dev->hdrbuf[0] = 0;

  dev->sz = 0;

  size = usbtmc_read(dev, dev->hdrbuf, MAX_TRANSFER_LEN, 1);

  if((size < 2) || (size > MAX_TRANSFER_LEN))
  {
    dev->hdrbuf[0] = 0;

    dev->buf[0] = 0;

    return -2;
  }

  dev->hdrbuf[size] = 0;

  if(dev->hdrbuf[0] != '#')
  {
    if(dev->hdrbuf[size - 1] == '\n')
    {
      dev->hdrbuf[--size] = 0;
    }

    dev->buf = dev->hdrbuf;

    dev->sz = size;

    return dev->sz;
  }

  strncpy(blockhdr, dev->hdrbuf, 16);

  len = blockhdr[1] - '0';

  if((len < 1) || (len > 9))
  {
    blockhdr[31] = 0;

    return -3;
  }

  blockhdr[len + 2] = 0;

  size2 = atoi(blockhdr + 2) + len + 2;

  while(size < size2 && size<MAX_TRANSFER_LEN) // we did not get all the data
  {
    ssize_t read_size = usbtmc_read(dev, &dev->hdrbuf[size], MAX_TRANSFER_LEN - size, 0);
    if(read_size < 1) // timeout or error occurred
    {
      blockhdr[31] = 0;

      return -4;
    }

    size += read_size;
  }

  size--;  // remove the last character

  if(size < size2)
  {
    blockhdr[31] = 0;

    return -5;
  }

  dev->buf = dev->hdrbuf + len + 2;

  dev->sz = size2 - len - 2;

  return dev->sz;
}
























