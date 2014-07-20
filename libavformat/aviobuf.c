/*
 * buffered I/O
 * Copyright (c) 2000,2001 Fabrice Bellard
 *
 * This file is part of Libav.
 *
 * Libav is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * Libav is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with Libav; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "libavutil/crc.h"
#include "libavutil/dict.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/log.h"
#include "libavutil/opt.h"
#include "avformat.h"
#include "avio.h"
#include "avio_internal.h"
#include "internal.h"
#include "url.h"
#include <stdarg.h>
#include <unistd.h>

#define IO_BUFFER_SIZE 32768
#define INPUT_HISTORY_SIZE 10 // s

/**
 * Do seeks within this distance ahead of the current buffer by skipping
 * data instead of calling the protocol seek function, for seekable
 * protocols.
 */
#define SHORT_SEEK_THRESHOLD 4096

static void *ffio_url_child_next(void *obj, void *prev)
{
    AVIOContext *s = obj;
    return prev ? NULL : s->opaque;
}

static const AVClass *ffio_url_child_class_next(const AVClass *prev)
{
    return prev ? NULL : &ffurl_context_class;
}

#define OFFSET(x) offsetof(AVIOContext, x)
#define D AV_OPT_FLAG_DECODING_PARAM
static const AVOption ffio_url_options[] = {
    { "in_packet_size", "set max input packet size", OFFSET(max_packet_size), AV_OPT_TYPE_INT, {.i64 = 0 }, 0, INT_MAX, D },
    { "throttle", "set max input rate in Kib/s", OFFSET(throttle), AV_OPT_TYPE_INT, {.i64 = 0 }, 0, INT_MAX, D },
    { NULL },
};

const AVClass ffio_url_class = {
    .class_name = "AVIOContext",
    .item_name  = av_default_item_name,
    .version    = LIBAVUTIL_VERSION_INT,
    .option     = ffio_url_options,
    .child_next = ffio_url_child_next,
    .child_class_next = ffio_url_child_class_next,
};

static void fill_buffer(AVIOContext *s);
static int url_resetbuf(AVIOContext *s, int flags);

int ffio_init_context(AVIOContext *s,
                  unsigned char *buffer,
                  int buffer_size,
                  int write_flag,
                  void *opaque,
                  int (*read_packet)(void *opaque, uint8_t *buf, int buf_size),
                  int (*write_packet)(void *opaque, uint8_t *buf, int buf_size),
                  int64_t (*seek)(void *opaque, int64_t offset, int whence))
{
    s->buffer      = buffer;
    s->buffer_size = buffer_size;
    s->buf_ptr     = buffer;
    s->opaque      = opaque;

    url_resetbuf(s, write_flag ? AVIO_FLAG_WRITE : AVIO_FLAG_READ);

    s->write_packet    = write_packet;
    s->read_packet     = read_packet;
    s->seek            = seek;
    s->pos             = 0;
    s->must_flush      = 0;
    s->eof_reached     = 0;
    s->error           = 0;
    s->seekable        = seek ? AVIO_SEEKABLE_NORMAL : 0;
    s->max_packet_size = 0;
    s->update_checksum = NULL;

    if (!read_packet && !write_flag) {
        s->pos     = buffer_size;
        s->buf_end = s->buffer + buffer_size;
    }
    s->read_pause = NULL;
    s->read_seek  = NULL;

    return 0;
}

AVIOContext *avio_alloc_context(
                  unsigned char *buffer,
                  int buffer_size,
                  int write_flag,
                  void *opaque,
                  int (*read_packet)(void *opaque, uint8_t *buf, int buf_size),
                  int (*write_packet)(void *opaque, uint8_t *buf, int buf_size),
                  int64_t (*seek)(void *opaque, int64_t offset, int whence))
{
    AVIOContext *s = av_mallocz(sizeof(AVIOContext));
    if (!s)
        return NULL;
    ffio_init_context(s, buffer, buffer_size, write_flag, opaque,
                  read_packet, write_packet, seek);
    return s;
}

static void flush_buffer(AVIOContext *s)
{
    if (s->buf_ptr > s->buffer) {
        if (s->write_packet && !s->error) {
            int ret = s->write_packet(s->opaque, s->buffer,
                                      s->buf_ptr - s->buffer);
            if (ret < 0) {
                s->error = ret;
            }
        }
        if (s->update_checksum) {
            s->checksum     = s->update_checksum(s->checksum, s->checksum_ptr,
                                                 s->buf_ptr - s->checksum_ptr);
            s->checksum_ptr = s->buffer;
        }
        s->pos += s->buf_ptr - s->buffer;
    }
    s->buf_ptr = s->buffer;
}

void avio_w8(AVIOContext *s, int b)
{
    *s->buf_ptr++ = b;
    if (s->buf_ptr >= s->buf_end)
        flush_buffer(s);
}

void ffio_fill(AVIOContext *s, int b, int count)
{
    while (count > 0) {
        int len = FFMIN(s->buf_end - s->buf_ptr, count);
        memset(s->buf_ptr, b, len);
        s->buf_ptr += len;

        if (s->buf_ptr >= s->buf_end)
            flush_buffer(s);

        count -= len;
    }
}

void avio_write(AVIOContext *s, const unsigned char *buf, int size)
{
    while (size > 0) {
        int len = FFMIN(s->buf_end - s->buf_ptr, size);
        memcpy(s->buf_ptr, buf, len);
        s->buf_ptr += len;

        if (s->buf_ptr >= s->buf_end)
            flush_buffer(s);

        buf += len;
        size -= len;
    }
}

void avio_flush(AVIOContext *s)
{
    flush_buffer(s);
    s->must_flush = 0;
}

int64_t avio_seek(AVIOContext *s, int64_t offset, int whence)
{
    int64_t offset1;
    int64_t pos;
    int force = whence & AVSEEK_FORCE;
    whence &= ~AVSEEK_FORCE;

    if(!s)
        return AVERROR(EINVAL);

    pos = s->pos - (s->write_flag ? 0 : (s->buf_end - s->buffer));

    if (whence != SEEK_CUR && whence != SEEK_SET)
        return AVERROR(EINVAL);

    if (whence == SEEK_CUR) {
        offset1 = pos + (s->buf_ptr - s->buffer);
        if (offset == 0)
            return offset1;
        offset += offset1;
    }
    offset1 = offset - pos;
    if (!s->must_flush &&
        offset1 >= 0 && offset1 <= (s->buf_end - s->buffer)) {
        /* can do the seek inside the buffer */
        s->buf_ptr = s->buffer + offset1;
    } else if ((!s->seekable ||
               offset1 <= s->buf_end + SHORT_SEEK_THRESHOLD - s->buffer) &&
               !s->write_flag && offset1 >= 0 &&
              (whence != SEEK_END || force)) {
        while(s->pos < offset && !s->eof_reached)
            fill_buffer(s);
        if (s->eof_reached)
            return AVERROR_EOF;
        s->buf_ptr = s->buf_end + offset - s->pos;
    } else {
        int64_t res;

        if (s->write_flag) {
            flush_buffer(s);
            s->must_flush = 1;
        }
        if (!s->seek)
            return AVERROR(EPIPE);
        if ((res = s->seek(s->opaque, offset, SEEK_SET)) < 0)
            return res;
        if (!s->write_flag)
            s->buf_end = s->buffer;
        s->buf_ptr = s->buffer;
        s->pos = offset;
    }
    s->eof_reached = 0;
    return offset;
}

int64_t avio_size(AVIOContext *s)
{
    int64_t size;

    if (!s)
        return AVERROR(EINVAL);

    if (!s->seek)
        return AVERROR(ENOSYS);
    size = s->seek(s->opaque, 0, AVSEEK_SIZE);
    if (size < 0) {
        if ((size = s->seek(s->opaque, -1, SEEK_END)) < 0)
            return size;
        size++;
        s->seek(s->opaque, s->pos, SEEK_SET);
    }
    return size;
}

void avio_wl32(AVIOContext *s, unsigned int val)
{
    avio_w8(s, val);
    avio_w8(s, val >> 8);
    avio_w8(s, val >> 16);
    avio_w8(s, val >> 24);
}

void avio_wb32(AVIOContext *s, unsigned int val)
{
    avio_w8(s, val >> 24);
    avio_w8(s, val >> 16);
    avio_w8(s, val >> 8);
    avio_w8(s, val);
}

int avio_put_str(AVIOContext *s, const char *str)
{
    int len = 1;
    if (str) {
        len += strlen(str);
        avio_write(s, (const unsigned char *) str, len);
    } else
        avio_w8(s, 0);
    return len;
}

int avio_put_str16le(AVIOContext *s, const char *str)
{
    const uint8_t *q = str;
    int ret = 0;

    while (*q) {
        uint32_t ch;
        uint16_t tmp;

        GET_UTF8(ch, *q++, break;)
        PUT_UTF16(ch, tmp, avio_wl16(s, tmp); ret += 2;)
    }
    avio_wl16(s, 0);
    ret += 2;
    return ret;
}

int ff_get_v_length(uint64_t val)
{
    int i = 1;

    while (val >>= 7)
        i++;

    return i;
}

void ff_put_v(AVIOContext *bc, uint64_t val)
{
    int i = ff_get_v_length(val);

    while (--i > 0)
        avio_w8(bc, 128 | (val >> (7 * i)));

    avio_w8(bc, val & 127);
}

void avio_wl64(AVIOContext *s, uint64_t val)
{
    avio_wl32(s, (uint32_t)(val & 0xffffffff));
    avio_wl32(s, (uint32_t)(val >> 32));
}

void avio_wb64(AVIOContext *s, uint64_t val)
{
    avio_wb32(s, (uint32_t)(val >> 32));
    avio_wb32(s, (uint32_t)(val & 0xffffffff));
}

void avio_wl16(AVIOContext *s, unsigned int val)
{
    avio_w8(s, val);
    avio_w8(s, val >> 8);
}

void avio_wb16(AVIOContext *s, unsigned int val)
{
    avio_w8(s, val >> 8);
    avio_w8(s, val);
}

void avio_wl24(AVIOContext *s, unsigned int val)
{
    avio_wl16(s, val & 0xffff);
    avio_w8(s, val >> 16);
}

void avio_wb24(AVIOContext *s, unsigned int val)
{
    avio_wb16(s, val >> 8);
    avio_w8(s, val);
}

/* Input stream */

/* traffic history for input throttle */
static void t_queue_push(traffic_queue *queue, int64_t time, int traf)
{
    traffic* data;
    queue->len++;
    data = av_realloc(queue->data, sizeof(traffic)*queue->len);
    if (!data)
        return;
    queue->data = data;
    queue->data[queue->len-1].time = time;
    queue->data[queue->len-1].traf = traf;
}

static void t_queue_pop(traffic_queue *queue)
{
    int i;
    for (i = 0; i < queue->len; i++)  {
        if (queue->data[i].time < av_gettime()-INPUT_HISTORY_SIZE*1000000) {
            memmove(queue->data + i, queue->data + i + 1, sizeof(traffic)*(queue->len - i - 1));
            queue->len--;
        }
    }
}

/* input throttle */
static int64_t avio_throttle_rate(traffic_queue *queue, int64_t time)
{
    int64_t sum = 0;
    int i;
    for (i = 0; i < queue->len; i++) {
        if (queue->data[i].time > av_gettime()-time)
            sum += queue->data[i].traf;
    }
    return (double)sum/((double)time/1000000.0); // B/s
}

static int avio_throttle(AVIOContext *s)
{
    if (!s->throttle) return 0;
    t_queue_pop(&s->t_queue);
    // measure speed at multiple intervals to create an even transfer rate
    if (avio_throttle_rate(&s->t_queue, 0.001*1000000) > s->throttle*0x80
        || avio_throttle_rate(&s->t_queue, 0.01*1000000) > s->throttle*0x80
        || avio_throttle_rate(&s->t_queue, 0.1*1000000) > s->throttle*0x80
        || avio_throttle_rate(&s->t_queue, 1*1000000) > s->throttle*0x80
        || avio_throttle_rate(&s->t_queue, 10*1000000) > s->throttle*0x80
        )
    return 1;
    return 0;
}

static void fill_buffer(AVIOContext *s)
{
    uint8_t *dst        = !s->max_packet_size &&
                          s->buf_end - s->buffer < s->buffer_size ?
                          s->buf_end : s->buffer;
    int len             = s->buffer_size - (dst - s->buffer);
    int max_buffer_size = s->max_packet_size ?
                          s->max_packet_size : IO_BUFFER_SIZE;

    /* can't fill the buffer without read_packet, just set EOF if appropriate */
    if (!s->read_packet && s->buf_ptr >= s->buf_end)
        s->eof_reached = 1;

    /* no need to do anything if EOF already reached */
    if (s->eof_reached)
        return;

    if (s->update_checksum && dst == s->buffer) {
        if (s->buf_end > s->checksum_ptr)
            s->checksum = s->update_checksum(s->checksum, s->checksum_ptr,
                                             s->buf_end - s->checksum_ptr);
        s->checksum_ptr = s->buffer;
    }

    /* make buffer smaller in case it ended up large after probing */
    if (s->buffer_size > max_buffer_size) {
        ffio_set_buf_size(s, max_buffer_size);

        s->checksum_ptr = dst = s->buffer;
        len = s->buffer_size;
    }

    if (s->read_packet)
        len = s->read_packet(s->opaque, dst, len);
    else
        len = 0;
    if (len <= 0) {
        /* do not modify buffer if EOF reached so that a seek back can
           be done without rereading data */
        s->eof_reached = 1;
        if (len < 0)
            s->error = len;
    } else {
        s->pos += len;
        s->buf_ptr = dst;
        s->buf_end = dst + len;
        
        // throttle
        t_queue_push(&s->t_queue, av_gettime(), len);
        while (s->throttle && avio_throttle(s)) {
            usleep(0.1*1000000);
        }
    }
}

unsigned long ff_crc04C11DB7_update(unsigned long checksum, const uint8_t *buf,
                                    unsigned int len)
{
    return av_crc(av_crc_get_table(AV_CRC_32_IEEE), checksum, buf, len);
}

unsigned long ff_crcA001_update(unsigned long checksum, const uint8_t *buf,
                                unsigned int len)
{
    return av_crc(av_crc_get_table(AV_CRC_16_ANSI_LE), checksum, buf, len);
}

unsigned long ffio_get_checksum(AVIOContext *s)
{
    s->checksum = s->update_checksum(s->checksum, s->checksum_ptr,
                                     s->buf_ptr - s->checksum_ptr);
    s->update_checksum = NULL;
    return s->checksum;
}

void ffio_init_checksum(AVIOContext *s,
                   unsigned long (*update_checksum)(unsigned long c, const uint8_t *p, unsigned int len),
                   unsigned long checksum)
{
    s->update_checksum = update_checksum;
    if (s->update_checksum) {
        s->checksum     = checksum;
        s->checksum_ptr = s->buf_ptr;
    }
}

/* XXX: put an inline version */
int avio_r8(AVIOContext *s)
{
    if (s->buf_ptr >= s->buf_end)
        fill_buffer(s);
    if (s->buf_ptr < s->buf_end)
        return *s->buf_ptr++;
    return 0;
}

int avio_read(AVIOContext *s, unsigned char *buf, int size)
{
    int len, size1;

    size1 = size;
    while (size > 0) {
        len = s->buf_end - s->buf_ptr;
        if (len > size)
            len = size;
        if (len == 0 || s->write_flag) {
            if(size > s->buffer_size && !s->update_checksum){
                if(s->read_packet)
                    len = s->read_packet(s->opaque, buf, size);
                if (len <= 0) {
                    /* do not modify buffer if EOF reached so that a seek back can
                    be done without rereading data */
                    s->eof_reached = 1;
                    if(len<0)
                        s->error= len;
                    break;
                } else {
                    s->pos += len;
                    size -= len;
                    buf += len;
                    s->buf_ptr = s->buffer;
                    s->buf_end = s->buffer/* + len*/;
                }
            } else {
                fill_buffer(s);
                len = s->buf_end - s->buf_ptr;
                if (len == 0)
                    break;
            }
        } else {
            memcpy(buf, s->buf_ptr, len);
            buf += len;
            s->buf_ptr += len;
            size -= len;
        }
    }
    if (size1 == size) {
        if (s->error)         return s->error;
        if (s->eof_reached)   return AVERROR_EOF;
    }
    return size1 - size;
}

int ffio_read_indirect(AVIOContext *s, unsigned char *buf, int size, const unsigned char **data)
{
    if (s->buf_end - s->buf_ptr >= size && !s->write_flag) {
        *data = s->buf_ptr;
        s->buf_ptr += size;
        return size;
    } else {
        *data = buf;
        return avio_read(s, buf, size);
    }
}

int ffio_read_partial(AVIOContext *s, unsigned char *buf, int size)
{
    int len;

    if (size < 0)
        return -1;

    if (s->read_packet && s->write_flag) {
        len = s->read_packet(s->opaque, buf, size);
        if (len > 0)
            s->pos += len;
        return len;
    }

    len = s->buf_end - s->buf_ptr;
    if (len == 0) {
        /* Reset the buf_end pointer to the start of the buffer, to make sure
         * the fill_buffer call tries to read as much data as fits into the
         * full buffer, instead of just what space is left after buf_end.
         * This avoids returning partial packets at the end of the buffer,
         * for packet based inputs.
         */
        s->buf_end = s->buf_ptr = s->buffer;
        fill_buffer(s);
        len = s->buf_end - s->buf_ptr;
    }
    if (len > size)
        len = size;
    memcpy(buf, s->buf_ptr, len);
    s->buf_ptr += len;
    if (!len) {
        if (s->error)         return s->error;
        if (s->eof_reached)   return AVERROR_EOF;
    }
    return len;
}

unsigned int avio_rl16(AVIOContext *s)
{
    unsigned int val;
    val = avio_r8(s);
    val |= avio_r8(s) << 8;
    return val;
}

unsigned int avio_rl24(AVIOContext *s)
{
    unsigned int val;
    val = avio_rl16(s);
    val |= avio_r8(s) << 16;
    return val;
}

unsigned int avio_rl32(AVIOContext *s)
{
    unsigned int val;
    val = avio_rl16(s);
    val |= avio_rl16(s) << 16;
    return val;
}

uint64_t avio_rl64(AVIOContext *s)
{
    uint64_t val;
    val = (uint64_t)avio_rl32(s);
    val |= (uint64_t)avio_rl32(s) << 32;
    return val;
}

unsigned int avio_rb16(AVIOContext *s)
{
    unsigned int val;
    val = avio_r8(s) << 8;
    val |= avio_r8(s);
    return val;
}

unsigned int avio_rb24(AVIOContext *s)
{
    unsigned int val;
    val = avio_rb16(s) << 8;
    val |= avio_r8(s);
    return val;
}
unsigned int avio_rb32(AVIOContext *s)
{
    unsigned int val;
    val = avio_rb16(s) << 16;
    val |= avio_rb16(s);
    return val;
}

int ff_get_line(AVIOContext *s, char *buf, int maxlen)
{
    int i = 0;
    char c;

    do {
        c = avio_r8(s);
        if (c && i < maxlen-1)
            buf[i++] = c;
    } while (c != '\n' && c);

    buf[i] = 0;
    return i;
}

int avio_get_str(AVIOContext *s, int maxlen, char *buf, int buflen)
{
    int i;

    if (buflen <= 0)
        return AVERROR(EINVAL);
    // reserve 1 byte for terminating 0
    buflen = FFMIN(buflen - 1, maxlen);
    for (i = 0; i < buflen; i++)
        if (!(buf[i] = avio_r8(s)))
            return i + 1;
    buf[i] = 0;
    for (; i < maxlen; i++)
        if (!avio_r8(s))
            return i + 1;
    return maxlen;
}

#define GET_STR16(type, read) \
    int avio_get_str16 ##type(AVIOContext *pb, int maxlen, char *buf, int buflen)\
{\
    char* q = buf;\
    int ret = 0;\
    if (buflen <= 0) \
        return AVERROR(EINVAL); \
    while (ret + 1 < maxlen) {\
        uint8_t tmp;\
        uint32_t ch;\
        GET_UTF16(ch, (ret += 2) <= maxlen ? read(pb) : 0, break;)\
        if (!ch)\
            break;\
        PUT_UTF8(ch, tmp, if (q - buf < buflen - 1) *q++ = tmp;)\
    }\
    *q = 0;\
    return ret;\
}\

GET_STR16(le, avio_rl16)
GET_STR16(be, avio_rb16)

#undef GET_STR16

uint64_t avio_rb64(AVIOContext *s)
{
    uint64_t val;
    val = (uint64_t)avio_rb32(s) << 32;
    val |= (uint64_t)avio_rb32(s);
    return val;
}

uint64_t ffio_read_varlen(AVIOContext *bc){
    uint64_t val = 0;
    int tmp;

    do{
        tmp = avio_r8(bc);
        val= (val<<7) + (tmp&127);
    }while(tmp&128);
    return val;
}

int ffio_fdopen(AVIOContext **s, URLContext *h)
{
    uint8_t *buffer;
    int buffer_size, max_packet_size;

    max_packet_size = h->max_packet_size;
    if (max_packet_size) {
        buffer_size = max_packet_size; /* no need to bufferize more than one packet */
    } else {
        buffer_size = IO_BUFFER_SIZE;
    }
    buffer = av_malloc(buffer_size);
    if (!buffer)
        return AVERROR(ENOMEM);

    *s = avio_alloc_context(buffer, buffer_size, h->flags & AVIO_FLAG_WRITE, h,
                            ffurl_read, ffurl_write, ffurl_seek);
    if (!*s) {
        av_free(buffer);
        return AVERROR(ENOMEM);
    }
    (*s)->seekable = h->is_streamed ? 0 : AVIO_SEEKABLE_NORMAL;
    (*s)->max_packet_size = max_packet_size;
    if(h->prot) {
        (*s)->read_pause = (int (*)(void *, int))h->prot->url_read_pause;
        (*s)->read_seek  = (int64_t (*)(void *, int, int64_t, int))h->prot->url_read_seek;
    }
    (*s)->av_class = &ffio_url_class;
    return 0;
}

int ffio_set_buf_size(AVIOContext *s, int buf_size)
{
    uint8_t *buffer;
    buffer = av_malloc(buf_size);
    if (!buffer)
        return AVERROR(ENOMEM);

    av_free(s->buffer);
    s->buffer = buffer;
    s->buffer_size = buf_size;
    s->buf_ptr = buffer;
    url_resetbuf(s, s->write_flag ? AVIO_FLAG_WRITE : AVIO_FLAG_READ);
    return 0;
}

static int url_resetbuf(AVIOContext *s, int flags)
{
    assert(flags == AVIO_FLAG_WRITE || flags == AVIO_FLAG_READ);

    if (flags & AVIO_FLAG_WRITE) {
        s->buf_end = s->buffer + s->buffer_size;
        s->write_flag = 1;
    } else {
        s->buf_end = s->buffer;
        s->write_flag = 0;
    }
    return 0;
}

int ffio_rewind_with_probe_data(AVIOContext *s, unsigned char *buf, int buf_size)
{
    int64_t buffer_start;
    int buffer_size;
    int overlap, new_size, alloc_size;

    if (s->write_flag)
        return AVERROR(EINVAL);

    buffer_size = s->buf_end - s->buffer;

    /* the buffers must touch or overlap */
    if ((buffer_start = s->pos - buffer_size) > buf_size)
        return AVERROR(EINVAL);

    overlap = buf_size - buffer_start;
    new_size = buf_size + buffer_size - overlap;

    alloc_size = FFMAX(s->buffer_size, new_size);
    if (alloc_size > buf_size)
        if (!(buf = av_realloc(buf, alloc_size)))
            return AVERROR(ENOMEM);

    if (new_size > buf_size) {
        memcpy(buf + buf_size, s->buffer + overlap, buffer_size - overlap);
        buf_size = new_size;
    }

    av_free(s->buffer);
    s->buf_ptr = s->buffer = buf;
    s->buffer_size = alloc_size;
    s->pos = buf_size;
    s->buf_end = s->buf_ptr + buf_size;
    s->eof_reached = 0;
    s->must_flush = 0;

    return 0;
}

int avio_open(AVIOContext **s, const char *filename, int flags)
{
    return avio_open2(s, filename, flags, NULL, NULL);
}

int avio_open2(AVIOContext **s, const char *filename, int flags,
               const AVIOInterruptCB *int_cb, AVDictionary **options)
{
    URLContext *h;
    int err;

    err = ffurl_open(&h, filename, flags, int_cb, options);
    if (err < 0)
        return err;
    err = ffio_fdopen(s, h);
    if (err < 0) {
        ffurl_close(h);
        return err;
    }
    return 0;
}

int avio_close(AVIOContext *s)
{
    URLContext *h;

    if (!s)
        return 0;

    avio_flush(s);
    h = s->opaque;
    av_freep(&s->buffer);
    av_free(s);
    return ffurl_close(h);
}

int avio_closep(AVIOContext **s)
{
    int ret = avio_close(*s);
    *s = NULL;
    return ret;
}

int avio_printf(AVIOContext *s, const char *fmt, ...)
{
    va_list ap;
    char buf[4096];
    int ret;

    va_start(ap, fmt);
    ret = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    avio_write(s, buf, strlen(buf));
    return ret;
}

int avio_pause(AVIOContext *s, int pause)
{
    if (!s->read_pause)
        return AVERROR(ENOSYS);
    return s->read_pause(s->opaque, pause);
}

int64_t avio_seek_time(AVIOContext *s, int stream_index,
                       int64_t timestamp, int flags)
{
    URLContext *h = s->opaque;
    int64_t ret;
    if (!s->read_seek)
        return AVERROR(ENOSYS);
    ret = s->read_seek(h, stream_index, timestamp, flags);
    if (ret >= 0) {
        int64_t pos;
        s->buf_ptr = s->buf_end; // Flush buffer
        pos = s->seek(h, 0, SEEK_CUR);
        if (pos >= 0)
            s->pos = pos;
        else if (pos != AVERROR(ENOSYS))
            ret = pos;
    }
    return ret;
}

/* output in a dynamic buffer */

typedef struct DynBuffer {
    int pos, size, allocated_size;
    uint8_t *buffer;
    int io_buffer_size;
    uint8_t io_buffer[1];
} DynBuffer;

static int dyn_buf_write(void *opaque, uint8_t *buf, int buf_size)
{
    DynBuffer *d = opaque;
    unsigned new_size, new_allocated_size;

    /* reallocate buffer if needed */
    new_size = d->pos + buf_size;
    new_allocated_size = d->allocated_size;
    if (new_size < d->pos || new_size > INT_MAX/2)
        return -1;
    while (new_size > new_allocated_size) {
        if (!new_allocated_size)
            new_allocated_size = new_size;
        else
            new_allocated_size += new_allocated_size / 2 + 1;
    }

    if (new_allocated_size > d->allocated_size) {
        int err;
        if ((err = av_reallocp(&d->buffer, new_allocated_size)) < 0) {
            d->allocated_size = 0;
            d->size = 0;
            return err;
        }
        d->allocated_size = new_allocated_size;
    }
    memcpy(d->buffer + d->pos, buf, buf_size);
    d->pos = new_size;
    if (d->pos > d->size)
        d->size = d->pos;
    return buf_size;
}

static int dyn_packet_buf_write(void *opaque, uint8_t *buf, int buf_size)
{
    unsigned char buf1[4];
    int ret;

    /* packetized write: output the header */
    AV_WB32(buf1, buf_size);
    ret = dyn_buf_write(opaque, buf1, 4);
    if (ret < 0)
        return ret;

    /* then the data */
    return dyn_buf_write(opaque, buf, buf_size);
}

static int64_t dyn_buf_seek(void *opaque, int64_t offset, int whence)
{
    DynBuffer *d = opaque;

    if (whence == SEEK_CUR)
        offset += d->pos;
    else if (whence == SEEK_END)
        offset += d->size;
    if (offset < 0 || offset > 0x7fffffffLL)
        return -1;
    d->pos = offset;
    return 0;
}

static int url_open_dyn_buf_internal(AVIOContext **s, int max_packet_size)
{
    DynBuffer *d;
    unsigned io_buffer_size = max_packet_size ? max_packet_size : 1024;

    if (sizeof(DynBuffer) + io_buffer_size < io_buffer_size)
        return -1;
    d = av_mallocz(sizeof(DynBuffer) + io_buffer_size);
    if (!d)
        return AVERROR(ENOMEM);
    d->io_buffer_size = io_buffer_size;
    *s = avio_alloc_context(d->io_buffer, d->io_buffer_size, 1, d, NULL,
                            max_packet_size ? dyn_packet_buf_write : dyn_buf_write,
                            max_packet_size ? NULL : dyn_buf_seek);
    if(!*s) {
        av_free(d);
        return AVERROR(ENOMEM);
    }
    (*s)->max_packet_size = max_packet_size;
    return 0;
}

int avio_open_dyn_buf(AVIOContext **s)
{
    return url_open_dyn_buf_internal(s, 0);
}

int ffio_open_dyn_packet_buf(AVIOContext **s, int max_packet_size)
{
    if (max_packet_size <= 0)
        return -1;
    return url_open_dyn_buf_internal(s, max_packet_size);
}

int avio_close_dyn_buf(AVIOContext *s, uint8_t **pbuffer)
{
    DynBuffer *d = s->opaque;
    int size;
    static const char padbuf[FF_INPUT_BUFFER_PADDING_SIZE] = {0};
    int padding = 0;

    if (!s) {
        *pbuffer = NULL;
        return 0;
    }

    /* don't attempt to pad fixed-size packet buffers */
    if (!s->max_packet_size) {
        avio_write(s, padbuf, sizeof(padbuf));
        padding = FF_INPUT_BUFFER_PADDING_SIZE;
    }

    avio_flush(s);

    *pbuffer = d->buffer;
    size = d->size;
    av_free(d);
    av_free(s);
    return size - padding;
}

static int null_buf_write(void *opaque, uint8_t *buf, int buf_size)
{
    DynBuffer *d = opaque;

    d->pos += buf_size;
    if (d->pos > d->size)
        d->size = d->pos;
    return buf_size;
}

int ffio_open_null_buf(AVIOContext **s)
{
    int ret = url_open_dyn_buf_internal(s, 0);
    if (ret >= 0) {
        AVIOContext *pb = *s;
        pb->write_packet = null_buf_write;
    }
    return ret;
}

int ffio_close_null_buf(AVIOContext *s)
{
    DynBuffer *d = s->opaque;
    int size;

    avio_flush(s);

    size = d->size;
    av_free(d);
    av_free(s);
    return size;
}
