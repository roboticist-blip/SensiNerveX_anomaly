/**
 * @file    ae_model.h
 * @brief   Sparse Autoencoder — bare-metal C, no framework
 *
 * Architecture (all ReLU activations except final decoder output which is linear):
 *   Encoder : x(10) → h1(8) [ReLU] → z(4)  [ReLU]
 *   Decoder : z(4)  → h2(8) [ReLU] → x̂(10) [Linear]
 *
 * Anomaly score = MSE(x, x̂)
 *
 * All weight arrays placed in CCM SRAM via .ccmram linker section for
 * maximum DMA-free bandwidth on STM32F405.
 */

#ifndef AE_MODEL_H
#define AE_MODEL_H

#include <stdint.h>
#include "snx_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AE_IN   SNX_AE_INPUT_DIM    /* 10 */
#define AE_H1   SNX_AE_H1_DIM      /*  8 */
#define AE_Z    SNX_AE_LATENT_DIM  /*  4 */
#define AE_H2   SNX_AE_H2_DIM      /*  8 */
#define AE_OUT  SNX_AE_OUTPUT_DIM  /* 10 */

typedef struct {

    float enc_W1[AE_H1][AE_IN];   /* (8,10) */
    float enc_b1[AE_H1];           /* (8)    */
    float enc_W2[AE_Z][AE_H1];    /* (4,8)  */
    float enc_b2[AE_Z];            /* (4)    */

    float dec_W1[AE_H2][AE_Z];    /* (8,4)  */
    float dec_b1[AE_H2];           /* (8)    */
    float dec_W2[AE_OUT][AE_H2];  /* (10,8) */
    float dec_b2[AE_OUT];          /* (10)   */
} AE_Weights;

typedef struct {
    float enc_W1[AE_H1][AE_IN];
    float enc_b1[AE_H1];
    float enc_W2[AE_Z][AE_H1];
    float enc_b2[AE_Z];
    float dec_W1[AE_H2][AE_Z];
    float dec_b1[AE_H2];
    float dec_W2[AE_OUT][AE_H2];
    float dec_b2[AE_OUT];
} AE_Velocity;

typedef struct {
    float h1[AE_H1];    
    float z[AE_Z];     
    float h2[AE_H2];    
    float x_hat[AE_OUT];
} AE_Activations;

typedef struct {
    float enc_W1[AE_H1][AE_IN];
    float enc_b1[AE_H1];
    float enc_W2[AE_Z][AE_H1];
    float enc_b2[AE_Z];
    float dec_W1[AE_H2][AE_Z];
    float dec_b1[AE_H2];
    float dec_W2[AE_OUT][AE_H2];
    float dec_b2[AE_OUT];
} AE_Gradients;

typedef struct {
    AE_Weights      w;
    AE_Velocity     v;
    AE_Activations  act;
    AE_Gradients    grad;
    uint32_t        train_steps;    
    float           last_mse;       
    uint8_t         is_trained;     
} AE_Model;


/**
 * @brief Xavier-initialise all weights; zero velocities and gradients.
 */
void AE_Init(AE_Model *m);

/**
 * @brief Forward pass: compute reconstruction x̂ and return MSE(x, x̂).
 * @param m   Model handle
 * @param x   Input feature vector [AE_IN]
 * @return    Reconstruction MSE (anomaly score)
 */
float AE_Forward(AE_Model *m, const float *x);

/**
 * @brief Backward pass: accumulate gradients for one sample.
 *        Call AE_Forward first to populate activations.
 * @param m   Model handle
 * @param x   Same input used in forward pass [AE_IN]
 */
void AE_Backward(AE_Model *m, const float *x);

/**
 * @brief Apply accumulated gradients via SGD+momentum, then zero gradients.
 * @param m          Model handle
 * @param lr         Learning rate
 * @param momentum   Momentum coefficient
 * @param l2_lambda  L2 regularisation coefficient
 * @param batch_size Divisor to average gradients
 */
void AE_Update(AE_Model *m, float lr, float momentum,
               float l2_lambda, uint32_t batch_size);

/**
 * @brief Zero all gradient accumulators.
 */
void AE_ZeroGrad(AE_Model *m);

/**
 * @brief Serialise weights to a flat float array for SD storage.
 *        buf must be >= AE_WEIGHT_FLAT_SIZE floats.
 */
#define AE_WEIGHT_FLAT_SIZE  (AE_H1*AE_IN + AE_H1 + AE_Z*AE_H1 + AE_Z + \
                              AE_H2*AE_Z  + AE_H2 + AE_OUT*AE_H2 + AE_OUT)
/* = 80+8+32+4+32+8+80+10 = 254 */

void AE_WeightsToFlat(const AE_Model *m, float *buf);

/**
 * @brief Deserialise weights from flat float array loaded from SD.
 */
void AE_WeightsFromFlat(AE_Model *m, const float *buf);

#ifdef __cplusplus
}
#endif

#endif /* AE_MODEL_H */
