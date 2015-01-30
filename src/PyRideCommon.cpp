//
//  PyRideCommon.cpp
//  PyRIDE
//
//  Created by Xun Wang on 17/08/12.
//
//

#include <openssl/sha.h>

#include "PyRideCommon.h"

#ifdef USE_ENCRYPTION

#include <openssl/crypto.h>
#include <openssl/bio.h>
#include <openssl/evp.h>
#include <openssl/buffer.h>
#include <openssl/err.h>

#ifdef WIN32
#include <WinBase.h>
#else
#include <pthread.h>
#endif

#define ENCRYPTION_KEY_LENGTH  32
#define PYRIDE_MSG_ENDECRYPT_BUFFER_SIZE       (PYRIDE_MSG_BUFFER_SIZE - 4)

/* Use the following python code to generate random 32 char key and encode into base64
 *
 * import base64
 * import string
 * import random
 *
 * def id_generator(size=6, chars=string.ascii_uppercase + string.ascii_lowercase + string.digits):
 *     return ''.join(random.choice(chars) for _ in range(size))
 *
 * key = id_generator(32)
 * base64.b64encode( key )
 */

static const unsigned char encrypt_key_text[] = "QWl3QnNQaFR5bGZCV3AyS3FiMlNuc0VpNng3SmswUU4=";
static const unsigned char encrypt_iv[] = "2xr4hWQl"; //must be 8 bytes
static unsigned char * encrypt_key;
static unsigned char * encryptbuffer = NULL;
static unsigned char * decryptbuffer = NULL;

#ifdef WIN32
static  CRITICAL_SECTION t_criticalSection_1_;
static  CRITICAL_SECTION t_criticalSection_2_;
#else
static pthread_mutex_t t_mutex_1;
static pthread_mutex_t t_mutex_2;
static pthread_mutexattr_t t_mta;
#endif

// helper functions
/* NOTE: encodeBase64 and decodeBase64 are not thread safe!!!! */
unsigned char * decodeBase64( const char * input, size_t * outLen )
{
  BIO * bmem, * b64;
  
  int inLen = (int)strlen( input );
  int maxOutLen=(inLen * 6 + 7) / 8;
  unsigned char * buf = (unsigned char *)malloc( maxOutLen );
  if (buf) {
    memset( buf, 0, maxOutLen );
    
    b64 = BIO_new( BIO_f_base64() );
    if (b64) {
      BIO_set_flags( b64, BIO_FLAGS_BASE64_NO_NL );
      bmem = BIO_new_mem_buf( (char *) input, inLen );
      b64 = BIO_push( b64, bmem );
      *outLen = BIO_read( b64, buf, maxOutLen );
      BIO_free_all( b64 );
    }
  }
  return buf;
}

char * encodeBase64( const unsigned char * input, size_t length )
{
  BIO * bmem, * b64;
  BUF_MEM * bptr;
  char * buf = NULL;
  
  b64 = BIO_new( BIO_f_base64() );
  if (b64) {
    BIO_set_flags( b64, BIO_FLAGS_BASE64_NO_NL );
    bmem = BIO_new( BIO_s_mem() );
    b64 = BIO_push( b64, bmem );
    BIO_write( b64, input, (int)length );
    BIO_flush( b64 );
    BIO_get_mem_ptr( b64, &bptr );
    
    buf = (char *)malloc( bptr->length + 1);
    memcpy( buf, bptr->data, bptr->length );
    buf[bptr->length] = 0;
    BIO_free_all( b64 );
  }
  return buf;
}

void endecryptInit()
{
  INFO_MSG( "Communication encryption enabled.\n" );

#ifdef WIN32
  InitializeCriticalSection( &t_criticalSection_1_ );
  InitializeCriticalSection( &t_criticalSection_2_ );
#else
  pthread_mutexattr_init( &t_mta );
  pthread_mutexattr_settype( &t_mta, PTHREAD_MUTEX_RECURSIVE );
  pthread_mutex_init( &t_mutex_1, &t_mta );
  pthread_mutex_init( &t_mutex_2, &t_mta );
#endif

  size_t keyLen = 0;
  if (encrypt_key) {
    free( encrypt_key );
  }
  encrypt_key = decodeBase64( (char *)encrypt_key_text, &keyLen );
  
  if (keyLen != ENCRYPTION_KEY_LENGTH) {
    //ERROR_MSG( "Encryption key decode error.\n" );
  }
  
  if (!encryptbuffer) {
    encryptbuffer = (unsigned char *)malloc( PYRIDE_MSG_ENDECRYPT_BUFFER_SIZE );
  }
  if (!decryptbuffer) {
    decryptbuffer = (unsigned char *)malloc( PYRIDE_MSG_ENDECRYPT_BUFFER_SIZE );
  }
}

void endecryptFini()
{
  if (encryptbuffer) {
    free( encryptbuffer );
    encryptbuffer = NULL;
  }
  if (decryptbuffer) {
    free( decryptbuffer );
    decryptbuffer = NULL;
  }
  if (encrypt_key) {
    free( encrypt_key );
    encrypt_key = NULL;
  }
#ifdef WIN32
  DeleteCriticalSection( &t_criticalSection_1_ );
  DeleteCriticalSection( &t_criticalSection_2_ );
#else
  pthread_mutex_destroy( &t_mutex_1 );
  pthread_mutex_destroy( &t_mutex_2 );
  pthread_mutexattr_destroy( &t_mta );
#endif
}

int decryptMessage( const unsigned char * origMesg, int origMesgLength, unsigned char ** decryptedMesg, int * decryptedMesgLength )
{
#ifdef WIN32
  EnterCriticalSection( &t_criticalSection_1_ );
#else
  pthread_mutex_lock( &t_mutex_1 );
#endif
  int oLen = 0, tLen = 0;
  EVP_CIPHER_CTX ctx;
  EVP_CIPHER_CTX_init( &ctx );
  EVP_DecryptInit( &ctx, EVP_bf_cbc(), encrypt_key, encrypt_iv );

  memset( decryptbuffer, 0, PYRIDE_MSG_ENDECRYPT_BUFFER_SIZE );
  
  if (EVP_DecryptUpdate( &ctx, decryptbuffer, &oLen, origMesg, origMesgLength ) != 1) {
    //ERROR_MSG( "EVP_DecryptUpdate failed.\n" );
    EVP_CIPHER_CTX_cleanup( &ctx );
#ifdef WIN32
    LeaveCriticalSection( &t_criticalSection_1_ );
#else
    pthread_mutex_unlock( &t_mutex_1 );
#endif
    return 0;
  }
  if (EVP_DecryptFinal( &ctx, decryptbuffer+oLen, &tLen ) != 1) {
    //ERROR_MSG( "EVP_DecryptFinal failed.\n" );
    EVP_CIPHER_CTX_cleanup( &ctx );
#ifdef WIN32
    LeaveCriticalSection( &t_criticalSection_1_ );
#else
    pthread_mutex_unlock( &t_mutex_1 );
#endif
    return 0;
  }
  
  *decryptedMesgLength = oLen + tLen;
  *decryptedMesg = decryptbuffer;
  EVP_CIPHER_CTX_cleanup( &ctx );
#ifdef WIN32
  LeaveCriticalSection( &t_criticalSection_1_ );
#else
  pthread_mutex_unlock( &t_mutex_1 );
#endif
  return 1;
}

int encryptMessage( const unsigned char * origMesg, int origMesgLength, unsigned char ** encryptedMesg, int * encryptedMesgLength )
{
#ifdef WIN32
  EnterCriticalSection( &t_criticalSection_2_ );
#else
  pthread_mutex_lock( &t_mutex_2 );
#endif
  int oLen = 0, tLen = 0;
  EVP_CIPHER_CTX ctx;
  EVP_CIPHER_CTX_init( &ctx );
  EVP_EncryptInit( &ctx, EVP_bf_cbc(), encrypt_key, encrypt_iv );
  
  memset( encryptbuffer, 0, PYRIDE_MSG_ENDECRYPT_BUFFER_SIZE );
  
  if (EVP_EncryptUpdate( &ctx, encryptbuffer, &oLen, origMesg, origMesgLength ) != 1) {
    //ERROR_MSG( "EVP_EncryptUpdate failed.\n" );
    EVP_CIPHER_CTX_cleanup( &ctx );
#ifdef WIN32
    LeaveCriticalSection( &t_criticalSection_2_ );
#else
    pthread_mutex_unlock( &t_mutex_2 );
#endif
    return 0;
  }
  
  if (EVP_EncryptFinal( &ctx, encryptbuffer+oLen, &tLen ) != 1) {
    //ERROR_MSG( "EVP_EncryptFinal failed.\n" );
    EVP_CIPHER_CTX_cleanup( &ctx );
#ifdef WIN32
    LeaveCriticalSection( &t_criticalSection_2_ );
#else
    pthread_mutex_unlock( &t_mutex_2 );
#endif
    return 0;
  }
  
  *encryptedMesgLength = oLen + tLen;
  *encryptedMesg = encryptbuffer;
  EVP_CIPHER_CTX_cleanup( &ctx );
#ifdef WIN32
  LeaveCriticalSection( &t_criticalSection_2_ );
#else
  pthread_mutex_unlock( &t_mutex_2 );
#endif
  return 1;
}

#endif // USE_ENCRYPTION
static const unsigned char hash_salt[] = "&cr1P";

int secureSHA256Hash( const unsigned char * password, const int pwlen, unsigned char * code )
{
  unsigned char * buf = NULL;
  if (!(password && code && pwlen > 0)) {
    return -1;
  }
  int buflen = SHA256_DIGEST_LENGTH + sizeof( hash_salt );
  buf = (unsigned char*) malloc( buflen );
  memset( buf, 0, buflen );
  SHA256( (unsigned char*)password, pwlen, buf );
  for (int i = 0; i < 100; i++) {
    memcpy( buf+SHA256_DIGEST_LENGTH, (void*)&hash_salt, sizeof(hash_salt));
    SHA256( buf, buflen, buf );
    
  }
  memcpy( code, buf, SHA256_DIGEST_LENGTH );
  free( buf );
  return 0;
}

#ifdef WIN32
#define FACTOR 0x19db1ded53e8000 
int win_gettimeofday( struct timeval * tp,void * tz )
{
  FILETIME f;
  ULARGE_INTEGER ifreq;
  LONGLONG res; 
  GetSystemTimeAsFileTime(&f);
  ifreq.HighPart = f.dwHighDateTime;
  ifreq.LowPart = f.dwLowDateTime;

  res = ifreq.QuadPart - FACTOR;
  tp->tv_sec = (long)((LONGLONG)res/10000000);
  tp->tv_usec = (long)((LONGLONG)res% 10000000000); // Micro Seonds

  return 0;
}
#endif