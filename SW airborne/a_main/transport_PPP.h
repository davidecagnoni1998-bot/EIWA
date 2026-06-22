/*
 * transport_PPP.h
 *
 *  Gestione Point to Point Protocol di trasporto
 *
 *  Created on: giugno 2020
 *      Author: Andrea Zaffino
 */

#ifndef TRANSPORT_PPP_H_
#define TRANSPORT_PPP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#define MAX_PPP_BUF_SIZE 128

typedef enum
{
    DATA_INCOMPLETE = 0,        /* nessun dato ricevuto o i dati ricevuti sono incompleti(fine trama non ricevuto) */
    SUCCESS_END = 1,            /* i dati ricevuti sono corretti */
    SUCCESS_DATA_PENDING = 2,   /* i dati ricevuti sono corretti ma sono presenti altri dati nel buffer */
    FAIL_CRC = -1,              /* i dati ricevuti sono incorretti(crc errato) */
    PARAMETER_ERROR = -2        /* un parametro passato alla funzione è errato */
} e_PPP_extract_result;

bool PPP_insert(const uint8_t* buf_in, const uint16_t size_in, uint8_t* buf_out, uint16_t* size_out);
e_PPP_extract_result PPP_extract(const uint8_t* buf_in, const uint16_t size_in, uint8_t* buf_out, uint16_t* size_out);

#ifdef __cplusplus
}
#endif

#endif /* TRANSPORT_PPP_H_ */
