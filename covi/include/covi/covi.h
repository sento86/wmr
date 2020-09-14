/*
 * covi.h
 *
 *  Created on: 09/09/2020
 *      Author: Vicent Girbes Juan
 */

#ifndef COVI_H_
#define COVI_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef M_PI
#define M_PI (3.141516F)
#endif

#include <cstdint>

typedef struct INPUT_DATA
{
    int16_t cpsL;
    int16_t cpsR;
}INPUT_DATA_T;

typedef struct SETUP_DATA
{
    int16_t mode;
    int16_t kp;
    int16_t kd;
    int16_t ki;
}SETUP_DATA_T;

typedef struct OUTPUT_DATA
{
    int16_t pwmL;
    int16_t pwmR;
}OUTPUT_DATA_T;

typedef struct STATE_DATA
{
    int16_t mode;
}STATE_DATA_T;

#ifdef __cplusplus
}
#endif

#endif /* COVI_H_ */
