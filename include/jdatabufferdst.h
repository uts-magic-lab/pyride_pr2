/*
 *  jdatastreamdst.h
 *  PyRIDE
 *
 *  Created by Xun Wang on 3/10/09.
 *  Copyright 2009 Galaxy Network. All rights reserved.
 *
 */

#ifndef JDATASTREAM_DST_H
#define JDATASTREAM_DST_H

#include <stdio.h>
#include "jpeglib.h"
#include "jerror.h"

#define MAX_JPEG_BUFFER_SIZE 921600
#define DEFAULT_IMAGE_SIZE  28672	/* choose an efficiently fwrite'able size */

typedef struct {
  struct jpeg_destination_mgr pub; /* public fields */

  int total_buffer_size;		/* total allocated buffer size */
  JOCTET * buffer;		/* start of buffer */
} buffer_destination_mgr;

typedef buffer_destination_mgr * buffer_dest_ptr;

void init_destination( j_compress_ptr cinfo );

boolean empty_output_buffer( j_compress_ptr cinfo );

void term_destination( j_compress_ptr cinfo );

void jpeg_databuffer_dest( j_compress_ptr cinfo );

unsigned char * get_jpeg_data_and_size( j_compress_ptr cinfo, int * data_size );

void jpeg_databuffer_free( j_compress_ptr cinfo );

void reset_jpeg_data_buffer( j_compress_ptr cinfo );

#endif // JDATASTREAM_DST_H
