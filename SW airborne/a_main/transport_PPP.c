/*
 * transport_PPP.cpp
 *
 *  Gestione Point to Point Protocol di trasporto
 *
 *  Created on: giugno 2020
 *      Author: Andrea Zaffino
 */

#include "transport_PPP.h"

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

/* definizioni locali */
#define ESCAPE 0x7D
#define SEOF 0x7E       /* start or end of frame */

typedef enum
{
    PPP_SEARCH_SOF,
    PPP_FRAME,
    PPP_ESCAPE_RECEIVED
} e_PPP_state;

typedef union
{
    struct
    {
        uint16_t length;
        uint8_t payload[MAX_PPP_BUF_SIZE - sizeof(uint16_t)];
    } field;
    uint8_t buffer[MAX_PPP_BUF_SIZE];      /* nel protocollo di trasporto viene incluso anche il crc nello stesso buffer, non compreso in length */
} frame_t;

static uint16_t crc_err_cnt = 0;
/* static uint16_t bufov_err_cnt = 0; */

/* Variabili e funzioni locali */
static uint16_t get_CRC(const uint8_t* data, int length);

/* Variabili e funzioni pubbliche */

// #define USE_MALLOC
#ifndef USE_MALLOC
uint8_t buffer_in_insert[MAX_PPP_BUF_SIZE * 2];
#endif  // USE_MALLOC

/**
* @brief  Riceve i dati da inserire nel protocollo di trasporto
* @param  buf_in: buffer contentente i dati da incapsulare nel protocollo di trasporto
* @param  size_in: lunghezza di buf_in
* @param  buf_out: buffer contentente i dati da incapsulati nel protocollo di trasporto
* @param  size_out: lunghezza di buf_out
* @retval false se buffer in ingresso troppo lungo, true buffer inserito correttamente
*/
bool PPP_insert(const uint8_t* buf_in, const uint16_t size_in, uint8_t* buf_out, uint16_t* size_out)
{
    uint16_t crc = 0u;
    uint16_t in_buf_len = size_in + (size_t)sizeof(crc);
    bool ret_val = false;

#ifdef USE_MALLOC
    uint8_t* buffer_in_insert = NULL;
    buffer_in_insert = (uint8_t*)malloc((size_t)in_buf_len);                /* buffer contenente buf_in e crc */
    if (buffer_in_insert != NULL)
    {
#endif  // USE_MALLOC
        if (in_buf_len <= sizeof(buffer_in_insert))
        {
            memcpy(buffer_in_insert, buf_in, (size_t)size_in);

            crc = get_CRC(buf_in, size_in);                                     /* calcola il crc della trama da inviare */
            buffer_in_insert[size_in] = crc >> 8;                               /* aggiunge il crc al fondo */
            buffer_in_insert[size_in + 1] = crc & 0xFF;

            *size_out = 0;
            buf_out[(*size_out)++] = SEOF;       /* 0x7E iniziale */
            for (uint16_t i = 0u; i < in_buf_len; i++)
            {
                if ((buffer_in_insert[i] == ESCAPE) || (buffer_in_insert[i] == SEOF))
                {
                    buf_out[(*size_out)++] = ESCAPE;
                    buf_out[(*size_out)++] = (uint8_t)(buffer_in_insert[i] ^ 0x20);
                }
                else
                {
                    buf_out[(*size_out)++] = buffer_in_insert[i];
                }
            }
            buf_out[(*size_out)++] = SEOF;       /* 0x7E finale */
            ret_val = true;
        }


#ifdef USE_MALLOC
        free(buffer_in_insert);
        buffer_in_insert = NULL;
    }
#endif  // USE_MALLOC
    return ret_val;
}

uint8_t buffer_in_extract[MAX_PPP_BUF_SIZE * 2];

/**
* @brief  Riceve i dati inseriti nel protocollo di trasporto e li estrae
* @param  buf_in: buffer contentente i dati incapsulati nel protocollo di trasporto
* @param  size_in: lunghezza di buf_in
* @param  buf_out: buffer contentente i dati da estratti dal protocollo di trasporto
* @param  size_out: lunghezza di buf_out, se passato valore diverso da 0u lo considera come lunghezza del buffer di uscita, altrimenti ipotizza sia uguale a MAX_PPP_BUF_SIZE
* @retval 0  : nessun dato ricevuto
*         1  : i dati ricevuti sono corretti
*         2  : i dati ricevuti sono corretti ma sono presenti altri dati nel buffer
*              In questo caso viene consegnata la prima trama e per ricavare la successiva e' necessario chiamare nuovamente questa funzione
*         -1 : i dati ricevuti sono incorretti (crc errato)
*         -2 : un parametro passato alla funzione è errato
*/
e_PPP_extract_result PPP_extract(const uint8_t* buf_in, const uint16_t size_in, uint8_t* buf_out, uint16_t* size_out)
{
    static uint16_t tot_size_in = 0u;
    e_PPP_state rx_state = PPP_SEARCH_SOF;
    e_PPP_extract_result ret_val = DATA_INCOMPLETE;

    /* ogni volta che a funzione viene chiamata i dati partono da inizio buffer */
    bool exit = false;
    uint16_t buf_out_pos = 0u;
    uint16_t max_buf_out = MAX_PPP_BUF_SIZE;
    if (*size_out != 0u)
    {
        max_buf_out = *size_out;
    }
    *size_out = 0u;

    if (size_in <= sizeof(buffer_in_extract))
    {
        /* accoda il buffer ricevuto all'eventuale precedente */
        if ((tot_size_in + size_in) > sizeof(buffer_in_extract))
        {
            // buffer esaurito, riparte da capo considerando i dati piu recenti
            tot_size_in = 0u;
        }

        if (size_in > 0u)
        {
            /* la funzione può essere chiamata ripetutamente anche con lunghezza in ingresso a 0,
               utile se la volta precendete ha restituito SUCCESS_DATA_PENDING */
            memcpy(&buffer_in_extract[tot_size_in], buf_in, (size_t)size_in);   /* TODO: verificare come si comporta memcpy nel caso di lunghezza a 0, se non da errore eliminare l'if */
        }
        tot_size_in += size_in;     /* totale */

        for (uint16_t buffer_in_pos = 0u; buffer_in_pos < tot_size_in; buffer_in_pos++)
        {
            switch (rx_state)
            {
                case PPP_SEARCH_SOF:
                    /* cerca l'inizio della trama */
                    if (SEOF == buffer_in_extract[buffer_in_pos])
                    {
                        if (ret_val == SUCCESS_END)
                        {
                            /* altra trama presente nel buffer, che verrà analizzata alla chiamata successiva di questa funzione */
                            /* appena trova un altro SEOF esce dichiarando che ci sono altre trame (anche incomplete) nel buffer */
                            tot_size_in -= buffer_in_pos;
                            memcpy(&buffer_in_extract[0], &buffer_in_extract[buffer_in_pos], (size_t)tot_size_in);      /* sposta il buffer all'inizio */
                            // memmove(&buffer_in_extract[0], &buffer_in_extract[buffer_in_pos], (size_t)tot_size_in);
                            ret_val = SUCCESS_DATA_PENDING;
                            exit = true;
                        }
                        else
                        {
                            /* primo SEOF del buffer ricevuto */
                            rx_state = PPP_FRAME;
                            buf_out_pos = 0u;
                        }
                    }
                    break;
                case PPP_FRAME:
                    if (ESCAPE == buffer_in_extract[buffer_in_pos])
                    {
                        /* ricevuto un escape */
                        rx_state = PPP_ESCAPE_RECEIVED;
                    }
                    else
                    {
                        if (SEOF == buffer_in_extract[buffer_in_pos])
                        {
                            if (buf_out_pos > sizeof(uint16_t)) /* crc + almeno 1 carattere */
                            {
                                *size_out = buf_out_pos - sizeof(uint16_t);     /* dichiara solo il payload, senza crc (anche se nel buffer e' comunque presente) */
                                uint16_t crc = (buf_out[buf_out_pos - 2] << 8) + buf_out[buf_out_pos - 1];
                                if (get_CRC(buf_out, *size_out) == crc)
                                {
                                    /***** trama ricevuta correttamente, invia solo il payload, senza crc *****/
                                    rx_state = PPP_SEARCH_SOF;
                                    ret_val = SUCCESS_END;
                                }
                                else
                                {
                                    crc_err_cnt++;  /* per debug */
                                    
                                    *size_out = 0u;
                                    buf_out_pos = 0u;       /* come se fosse il primo SEOF incontrato */
                                    ret_val = FAIL_CRC;
                                }
                            }
                            else
                            {
                                /* se dopo meno di 3 caratteri (crc + 1) riceve un altro SEOF e' stata persa una trama */
                                buf_out_pos = 0;
                            }
                        }
                        else
                        {
                            buf_out[buf_out_pos++] = buffer_in_extract[buffer_in_pos];
                        }
                    }
                    break;
                case PPP_ESCAPE_RECEIVED:
                    /* ricevuto escape */
                    if (((SEOF ^ 0x20) == buffer_in_extract[buffer_in_pos]) || ((ESCAPE ^ 0x20) == buffer_in_extract[buffer_in_pos]))
                    {
                        buf_out[buf_out_pos++] = buffer_in_extract[buffer_in_pos] ^ 0x20;       /* viene sostituito con SEOF o ESCAPE */
                        rx_state = PPP_FRAME;
                    }
                    else
                    {
                        /* carattere errato dopo l'escape */
                        rx_state = PPP_SEARCH_SOF;
                    }
                    break;
                default:
                    /* stato non gestito */
                    rx_state = PPP_SEARCH_SOF;
                    break;
            }   /* switch end */

            if (buf_out_pos >= max_buf_out)
            {
                /* buffer di output passato non sufficiente */
                ret_val = PARAMETER_ERROR;
                exit = true;
            }

            if (exit == true)
            {
                break;
            }
        }

        if ((ret_val != SUCCESS_DATA_PENDING) && (rx_state != PPP_FRAME))
        {
            /* pulisce il numero di dati rimanenti se non c'è ancora una trama */
            tot_size_in = 0u;
        }
    }
    else
    {
        ret_val = PARAMETER_ERROR;
    }

    return ret_val;
}

/**
* @brief  Ricezione trama da pc, contenente i dati applicativi
* @param  data: puntatore al buffer contentente i dati su cui calcolare il crc
* @param  length: del buffer contentente i dati su cui calcolare il crc
* @retval uint16_t: CRC
*/
static uint16_t get_CRC(const uint8_t* data, int length)
{
    uint16_t crc = 0;
    /* offset e' il primo carattere (STX) */
    for (uint16_t i = 0; length > 0; i++, length--)
    {
        crc ^= data[i];
    }
    uint8_t b0 = (uint8_t)((crc >> 4 & 0x0F) + '0');
    if (b0 > (uint8_t)'9')
    {
        b0 += 0x07;
    }
    uint8_t b1 = (uint8_t)((crc & 0x0F) + '0');
    if (b1 > (uint8_t)'9')
    {
        b1 += 0x07;
    }

    return b0 + (b1 << 8);
}
