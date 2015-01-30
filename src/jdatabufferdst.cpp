/*
 * jdatabufferdst.c
 *
 * Modified based on the standard io destination code found
 * in libjpeg library code.
 */

/* this is not a core library module, so it doesn't define JPEG_INTERNALS */
#include <stdlib.h>

#include "jdatabufferdst.h"

/*
 * Initialize destination --- called by jpeg_start_compress
 * before any data is actually written.
 */

void init_destination( j_compress_ptr cinfo )
{
  buffer_dest_ptr dest = (buffer_dest_ptr)cinfo->dest;

  dest->pub.next_output_byte = dest->buffer;
  dest->pub.free_in_buffer = DEFAULT_IMAGE_SIZE;
}

/*
 * Empty the output buffer --- called whenever buffer fills up.
 *
 * In typical applications, this should write the entire output buffer
 * (ignoring the current state of next_output_byte & free_in_buffer),
 * reset the pointer & count to the start of the buffer, and return TRUE
 * indicating that the buffer has been dumped.
 *
 * In applications that need to be able to suspend compression due to output
 * overrun, a FALSE return indicates that the buffer cannot be emptied now.
 * In this situation, the compressor will return to its caller (possibly with
 * an indication that it has not accepted all the supplied scanlines).  The
 * application should resume compression after it has made more room in the
 * output buffer.  Note that there are substantial restrictions on the use of
 * suspension --- see the documentation.
 *
 * When suspending, the compressor will back up to a convenient restart point
 * (typically the start of the current MCU). next_output_byte & free_in_buffer
 * indicate where the restart point will be if the current call returns FALSE.
 * Data beyond this point will be regenerated after resumption, so do not
 * write it out when emptying the buffer externally.
 */

boolean empty_output_buffer( j_compress_ptr cinfo )
{  
  buffer_dest_ptr dest = (buffer_dest_ptr) cinfo->dest;
  
  int old_buffer_size = dest->total_buffer_size;
  int new_buffer_size = old_buffer_size + DEFAULT_IMAGE_SIZE / 10;
  
  if (new_buffer_size > MAX_JPEG_BUFFER_SIZE) {
    return FALSE;
  }
  
  JOCTET * new_buffer = (JOCTET *)realloc( dest->buffer, new_buffer_size * sizeof( JOCTET ) );
  if (new_buffer == NULL) {
    return FALSE;
  }

  dest->pub.next_output_byte = new_buffer + old_buffer_size;
  dest->pub.free_in_buffer = new_buffer_size - old_buffer_size;
  dest->total_buffer_size = new_buffer_size;
  dest->buffer = new_buffer;

  return TRUE;
}

/*
 * Terminate destination --- called by jpeg_finish_compress
 * after all data has been written.  Usually needs to flush buffer.
 *
 * NB: *not* called by jpeg_abort or jpeg_destroy; surrounding
 * application must deal with any cleanup that should happen even
 * for error exit.
 */

void term_destination( j_compress_ptr cinfo )
{
}

/*
 * Prepare for output to a stdio stream.
 * The caller must have already opened the stream, and is responsible
 * for closing it after finishing compression.
 */

void jpeg_databuffer_dest( j_compress_ptr cinfo )
{
  buffer_dest_ptr dest;

  /* The destination object is made permanent so that multiple JPEG images
   * can be written to the same file without re-executing jpeg_stdio_dest.
   * This makes it dangerous to use this manager and a different destination
   * manager serially with the same JPEG object, because their private object
   * sizes may be different.  Caveat programmer.
   */
  if (cinfo->dest == NULL) {	/* first time for this JPEG object? */
    cinfo->dest = (struct jpeg_destination_mgr *)
      (*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_PERMANENT,
				  sizeof( buffer_destination_mgr ) );
  }

  dest = (buffer_dest_ptr) cinfo->dest;
  dest->pub.init_destination = init_destination;
  dest->pub.empty_output_buffer = empty_output_buffer;
  dest->pub.term_destination = term_destination;
  /* Allocate the output buffer --- it will be released when done with image */
  dest->buffer = (JOCTET *)malloc( DEFAULT_IMAGE_SIZE * sizeof( JOCTET ) );
  dest->total_buffer_size = DEFAULT_IMAGE_SIZE;
}

unsigned char * get_jpeg_data_and_size( j_compress_ptr cinfo, int * data_size )
{
  buffer_dest_ptr dest = (buffer_dest_ptr) cinfo->dest;

  *data_size = dest->pub.next_output_byte - dest->buffer;
  return (unsigned char *)dest->buffer;
}

void jpeg_databuffer_free( j_compress_ptr cinfo )
{
  buffer_dest_ptr dest = (buffer_dest_ptr) cinfo->dest;
  if (dest->buffer) {
    free( dest->buffer );
    dest->buffer = NULL;
  }
  dest->total_buffer_size = 0;
  dest->pub.free_in_buffer = 0;
  dest->pub.next_output_byte = NULL;
}

void reset_jpeg_data_buffer( j_compress_ptr cinfo )
{
  buffer_dest_ptr dest = (buffer_dest_ptr) cinfo->dest;
  dest->pub.free_in_buffer = dest->total_buffer_size;
  dest->pub.next_output_byte = dest->buffer;
}

