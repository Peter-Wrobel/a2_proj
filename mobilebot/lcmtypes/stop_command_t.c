// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

#include <string.h>
#include "stop_command_t.h"

static int __stop_command_t_hash_computed;
static uint64_t __stop_command_t_hash;

uint64_t __stop_command_t_hash_recursive(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __stop_command_t_get_hash)
            return 0;

    __lcm_hash_ptr cp;
    cp.parent =  p;
    cp.v = (void*)__stop_command_t_get_hash;
    (void) cp;

    uint64_t hash = (uint64_t)0x90e9a182e0b7ac28LL
         + __int8_t_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __stop_command_t_get_hash(void)
{
    if (!__stop_command_t_hash_computed) {
        __stop_command_t_hash = (int64_t)__stop_command_t_hash_recursive(NULL);
        __stop_command_t_hash_computed = 1;
    }

    return __stop_command_t_hash;
}

int __stop_command_t_encode_array(void *buf, int offset, int maxlen, const stop_command_t *p, int elements)
{
    int pos = 0, element;
    int thislen;

    for (element = 0; element < elements; element++) {

        thislen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].stop), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int stop_command_t_encode(void *buf, int offset, int maxlen, const stop_command_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __stop_command_t_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __stop_command_t_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int __stop_command_t_encoded_array_size(const stop_command_t *p, int elements)
{
    int size = 0, element;
    for (element = 0; element < elements; element++) {

        size += __int8_t_encoded_array_size(&(p[element].stop), 1);

    }
    return size;
}

int stop_command_t_encoded_size(const stop_command_t *p)
{
    return 8 + __stop_command_t_encoded_array_size(p, 1);
}

int __stop_command_t_decode_array(const void *buf, int offset, int maxlen, stop_command_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].stop), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int __stop_command_t_decode_array_cleanup(stop_command_t *p, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int8_t_decode_array_cleanup(&(p[element].stop), 1);

    }
    return 0;
}

int stop_command_t_decode(const void *buf, int offset, int maxlen, stop_command_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __stop_command_t_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __stop_command_t_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int stop_command_t_decode_cleanup(stop_command_t *p)
{
    return __stop_command_t_decode_array_cleanup(p, 1);
}

int __stop_command_t_clone_array(const stop_command_t *p, stop_command_t *q, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int8_t_clone_array(&(p[element].stop), &(q[element].stop), 1);

    }
    return 0;
}

stop_command_t *stop_command_t_copy(const stop_command_t *p)
{
    stop_command_t *q = (stop_command_t*) malloc(sizeof(stop_command_t));
    __stop_command_t_clone_array(p, q, 1);
    return q;
}

void stop_command_t_destroy(stop_command_t *p)
{
    __stop_command_t_decode_array_cleanup(p, 1);
    free(p);
}

int stop_command_t_publish(lcm_t *lc, const char *channel, const stop_command_t *p)
{
      int max_data_size = stop_command_t_encoded_size (p);
      uint8_t *buf = (uint8_t*) malloc (max_data_size);
      if (!buf) return -1;
      int data_size = stop_command_t_encode (buf, 0, max_data_size, p);
      if (data_size < 0) {
          free (buf);
          return data_size;
      }
      int status = lcm_publish (lc, channel, buf, data_size);
      free (buf);
      return status;
}

struct _stop_command_t_subscription_t {
    stop_command_t_handler_t user_handler;
    void *userdata;
    lcm_subscription_t *lc_h;
};
static
void stop_command_t_handler_stub (const lcm_recv_buf_t *rbuf,
                            const char *channel, void *userdata)
{
    int status;
    stop_command_t p;
    memset(&p, 0, sizeof(stop_command_t));
    status = stop_command_t_decode (rbuf->data, 0, rbuf->data_size, &p);
    if (status < 0) {
        fprintf (stderr, "error %d decoding stop_command_t!!!\n", status);
        return;
    }

    stop_command_t_subscription_t *h = (stop_command_t_subscription_t*) userdata;
    h->user_handler (rbuf, channel, &p, h->userdata);

    stop_command_t_decode_cleanup (&p);
}

stop_command_t_subscription_t* stop_command_t_subscribe (lcm_t *lcm,
                    const char *channel,
                    stop_command_t_handler_t f, void *userdata)
{
    stop_command_t_subscription_t *n = (stop_command_t_subscription_t*)
                       malloc(sizeof(stop_command_t_subscription_t));
    n->user_handler = f;
    n->userdata = userdata;
    n->lc_h = lcm_subscribe (lcm, channel,
                                 stop_command_t_handler_stub, n);
    if (n->lc_h == NULL) {
        fprintf (stderr,"couldn't reg stop_command_t LCM handler!\n");
        free (n);
        return NULL;
    }
    return n;
}

int stop_command_t_subscription_set_queue_capacity (stop_command_t_subscription_t* subs,
                              int num_messages)
{
    return lcm_subscription_set_queue_capacity (subs->lc_h, num_messages);
}

int stop_command_t_unsubscribe(lcm_t *lcm, stop_command_t_subscription_t* hid)
{
    int status = lcm_unsubscribe (lcm, hid->lc_h);
    if (0 != status) {
        fprintf(stderr,
           "couldn't unsubscribe stop_command_t_handler %p!\n", hid);
        return -1;
    }
    free (hid);
    return 0;
}

