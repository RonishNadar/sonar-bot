#ifndef KALMAN_H
#define KALMAN_H

class Kalman1D {
public:
    Kalman1D(float q = 0.01, float r = 0.5, float p = 1.0, float init = 0.0)
        : Q(q), R(r), P(p), X(init) {}

    void predict(float u = 0.0) {
        X += u;
        P += Q;
    }

    void update(float measurement) {
        float K = P / (P + R);
        X += K * (measurement - X);
        P *= (1 - K);
    }

    float get() const { return X; }

private:
    float Q;  // process noise
    float R;  // measurement noise
    float P;  // error covariance
    float X;  // estimate
};

#endif
