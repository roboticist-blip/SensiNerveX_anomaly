/**
 * @file    ae_model.c
 * @brief   Bare-metal Sparse Autoencoder — forward, backward, SGD update
 *
 * All weight/velocity/gradient structs are declared in CCM SRAM by the
 * caller (supervisor.c) via:
 *   __attribute__((section(".ccmram"))) AE_Model g_ae;
 *
 * No heap allocation, no stdlib math beyond sqrtf.
 */

#include "ae_model.h"
#include <string.h>   /* memset  */
#include <math.h>     /* sqrtf   */
#include <stdint.h>

static inline float relu(float x)       { return x > 0.0f ? x : 0.0f; }
static inline float relu_d(float x)     { return x > 0.0f ? 1.0f : 0.0f; }

static uint32_t s_lcg_state = 0xDEADBEEFu;

static float lcg_uniform_pm1(void)
{
    s_lcg_state = s_lcg_state * 1664525u + 1013904223u;
    return ((float)(int32_t)s_lcg_state) / (float)0x80000000u;
}

static void xavier_fill(float *arr, uint32_t rows, uint32_t cols)
{
    float limit = sqrtf(6.0f / (float)(rows + cols));
    uint32_t n = rows * cols;
    for (uint32_t i = 0; i < n; i++) {
        arr[i] = lcg_uniform_pm1() * limit;
    }
}

void AE_Init(AE_Model *m)
{
    memset(m, 0, sizeof(AE_Model));

    xavier_fill(&m->w.enc_W1[0][0], AE_H1, AE_IN);
    xavier_fill(&m->w.enc_W2[0][0], AE_Z,  AE_H1);
    xavier_fill(&m->w.dec_W1[0][0], AE_H2,  AE_Z);
    xavier_fill(&m->w.dec_W2[0][0], AE_OUT, AE_H2);

    m->train_steps = 0;
    m->last_mse    = 0.0f;
    m->is_trained  = 0;
}

float AE_Forward(AE_Model *m, const float *x)
{
    uint32_t i, j;
    float acc;

    for (i = 0; i < AE_H1; i++) {
        acc = m->w.enc_b1[i];
        for (j = 0; j < AE_IN; j++) acc += m->w.enc_W1[i][j] * x[j];
        m->act.h1[i] = relu(acc);
    }

    for (i = 0; i < AE_Z; i++) {
        acc = m->w.enc_b2[i];
        for (j = 0; j < AE_H1; j++) acc += m->w.enc_W2[i][j] * m->act.h1[j];
        m->act.z[i] = relu(acc);
    }

    for (i = 0; i < AE_H2; i++) {
        acc = m->w.dec_b1[i];
        for (j = 0; j < AE_Z; j++) acc += m->w.dec_W1[i][j] * m->act.z[j];
        m->act.h2[i] = relu(acc);
    }

    for (i = 0; i < AE_OUT; i++) {
        acc = m->w.dec_b2[i];
        for (j = 0; j < AE_H2; j++) acc += m->w.dec_W2[i][j] * m->act.h2[j];
        m->act.x_hat[i] = acc;
    }

    float mse = 0.0f;
    for (i = 0; i < AE_OUT; i++) {
        float diff = m->act.x_hat[i] - x[i];
        mse += diff * diff;
    }
    mse /= (float)AE_OUT;
    m->last_mse = mse;
    return mse;
}

/*
 * Backpropagation through MSE loss.
 * Gradients are ACCUMULATED into m->grad — call AE_ZeroGrad() before a new batch.
 *
 * dL/dx̂_i = (2/N)(x̂_i - x_i)      [MSE gradient at output]
 */
void AE_Backward(AE_Model *m, const float *x)
{
    uint32_t i, j;

    //δ at decoder output (linear, dL/dx̂) 
    float d_xhat[AE_OUT];
    float inv_N = 2.0f / (float)AE_OUT;
    for (i = 0; i < AE_OUT; i++) {
        d_xhat[i] = inv_N * (m->act.x_hat[i] - x[i]);
    }

    //Accumulate dec_W2 and dec_b2 gradients 
    for (i = 0; i < AE_OUT; i++) {
        m->grad.dec_b2[i] += d_xhat[i];
        for (j = 0; j < AE_H2; j++) {
            m->grad.dec_W2[i][j] += d_xhat[i] * m->act.h2[j];
        }
    }

    //δ at h2 (through dec_W2, then ReLU)
    float d_h2[AE_H2];
    for (j = 0; j < AE_H2; j++) {
        float s = 0.0f;
        for (i = 0; i < AE_OUT; i++) s += d_xhat[i] * m->w.dec_W2[i][j];
        d_h2[j] = s * relu_d(m->act.h2[j]);
    }

    //Accumulate dec_W1 and dec_b1 gradients
    for (i = 0; i < AE_H2; i++) {
        m->grad.dec_b1[i] += d_h2[i];
        for (j = 0; j < AE_Z; j++) {
            m->grad.dec_W1[i][j] += d_h2[i] * m->act.z[j];
        }
    }

    //δ at z (through dec_W1, then ReLU)
    float d_z[AE_Z];
    for (j = 0; j < AE_Z; j++) {
        float s = 0.0f;
        for (i = 0; i < AE_H2; i++) s += d_h2[i] * m->w.dec_W1[i][j];
        d_z[j] = s * relu_d(m->act.z[j]);
    }

    //Accumulate enc_W2 and enc_b2 gradients 
    for (i = 0; i < AE_Z; i++) {
        m->grad.enc_b2[i] += d_z[i];
        for (j = 0; j < AE_H1; j++) {
            m->grad.enc_W2[i][j] += d_z[i] * m->act.h1[j];
        }
    }

    //δ at h1 (through enc_W2, then ReLU)
    float d_h1[AE_H1];
    for (j = 0; j < AE_H1; j++) {
        float s = 0.0f;
        for (i = 0; i < AE_Z; i++) s += d_z[i] * m->w.enc_W2[i][j];
        d_h1[j] = s * relu_d(m->act.h1[j]);
    }

    //Accumulate enc_W1 and enc_b1 gradients
    for (i = 0; i < AE_H1; i++) {
        m->grad.enc_b1[i] += d_h1[i];
        for (j = 0; j < AE_IN; j++) {
            m->grad.enc_W1[i][j] += d_h1[i] * x[j];
        }
    }
}

void AE_ZeroGrad(AE_Model *m)
{
    memset(&m->grad, 0, sizeof(AE_Gradients));
}

/*
 * SGD with momentum + L2 regularisation.
 * v ← momentum*v - lr*(grad/batch + l2*w)
 * w ← w + v
 */
#define _UPDATE_ARRAY(W_arr, V_arr, G_arr, N)                       \
    do {                                                             \
        for (uint32_t _i = 0; _i < (N); _i++) {                    \
            float _g = (G_arr)[_i] / (float)batch_size             \
                       + l2_lambda * (W_arr)[_i];                   \
            (V_arr)[_i] = momentum * (V_arr)[_i] - lr * _g;        \
            (W_arr)[_i] += (V_arr)[_i];                            \
        }                                                            \
    } while (0)

void AE_Update(AE_Model *m, float lr, float momentum,
               float l2_lambda, uint32_t batch_size)
{
    _UPDATE_ARRAY(&m->w.enc_W1[0][0], &m->v.enc_W1[0][0], &m->grad.enc_W1[0][0], AE_H1 * AE_IN);
    _UPDATE_ARRAY(m->w.enc_b1, m->v.enc_b1, m->grad.enc_b1, AE_H1);
    _UPDATE_ARRAY(&m->w.enc_W2[0][0], &m->v.enc_W2[0][0], &m->grad.enc_W2[0][0], AE_Z * AE_H1);
    _UPDATE_ARRAY(m->w.enc_b2, m->v.enc_b2, m->grad.enc_b2, AE_Z);

    _UPDATE_ARRAY(&m->w.dec_W1[0][0], &m->v.dec_W1[0][0], &m->grad.dec_W1[0][0], AE_H2 * AE_Z);
    _UPDATE_ARRAY(m->w.dec_b1, m->v.dec_b1, m->grad.dec_b1, AE_H2);
    _UPDATE_ARRAY(&m->w.dec_W2[0][0], &m->v.dec_W2[0][0], &m->grad.dec_W2[0][0], AE_OUT * AE_H2);
    _UPDATE_ARRAY(m->w.dec_b2, m->v.dec_b2, m->grad.dec_b2, AE_OUT);

    m->train_steps++;
    m->is_trained = 1;
}

#undef _UPDATE_ARRAY

void AE_WeightsToFlat(const AE_Model *m, float *buf)
{
    uint32_t idx = 0;
    for (uint32_t i = 0; i < AE_H1; i++)
        for (uint32_t j = 0; j < AE_IN; j++)
            buf[idx++] = m->w.enc_W1[i][j];
    for (uint32_t i = 0; i < AE_H1; i++) buf[idx++] = m->w.enc_b1[i];
    for (uint32_t i = 0; i < AE_Z; i++)
        for (uint32_t j = 0; j < AE_H1; j++)
            buf[idx++] = m->w.enc_W2[i][j];
    for (uint32_t i = 0; i < AE_Z; i++) buf[idx++] = m->w.enc_b2[i];
    for (uint32_t i = 0; i < AE_H2; i++)
        for (uint32_t j = 0; j < AE_Z; j++)
            buf[idx++] = m->w.dec_W1[i][j];
    for (uint32_t i = 0; i < AE_H2; i++) buf[idx++] = m->w.dec_b1[i];
    for (uint32_t i = 0; i < AE_OUT; i++)
        for (uint32_t j = 0; j < AE_H2; j++)
            buf[idx++] = m->w.dec_W2[i][j];
    for (uint32_t i = 0; i < AE_OUT; i++) buf[idx++] = m->w.dec_b2[i];
}

void AE_WeightsFromFlat(AE_Model *m, const float *buf)
{
    uint32_t idx = 0;
    for (uint32_t i = 0; i < AE_H1; i++)
        for (uint32_t j = 0; j < AE_IN; j++)
            m->w.enc_W1[i][j] = buf[idx++];
    for (uint32_t i = 0; i < AE_H1; i++) m->w.enc_b1[i] = buf[idx++];
    for (uint32_t i = 0; i < AE_Z; i++)
        for (uint32_t j = 0; j < AE_H1; j++)
            m->w.enc_W2[i][j] = buf[idx++];
    for (uint32_t i = 0; i < AE_Z; i++) m->w.enc_b2[i] = buf[idx++];
    for (uint32_t i = 0; i < AE_H2; i++)
        for (uint32_t j = 0; j < AE_Z; j++)
            m->w.dec_W1[i][j] = buf[idx++];
    for (uint32_t i = 0; i < AE_H2; i++) m->w.dec_b1[i] = buf[idx++];
    for (uint32_t i = 0; i < AE_OUT; i++)
        for (uint32_t j = 0; j < AE_H2; j++)
            m->w.dec_W2[i][j] = buf[idx++];
    for (uint32_t i = 0; i < AE_OUT; i++) m->w.dec_b2[i] = buf[idx++];
}
