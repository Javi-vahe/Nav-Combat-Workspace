#ifndef SIMPLE_KALMAN_FILTER_HPP
#define SIMPLE_KALMAN_FILTER_HPP

class SimpleKalmanFilter
{
private:
    double Q; // 过程噪声
    double R; // 测量噪声
    double X; // 状态估计
    double P; // 估计误差协方差

public:
    SimpleKalmanFilter(double q, double r, double initial_value)
        : Q(q), R(r), X(initial_value), P(1.0) {}

    double update(double& measurement)
    {
        // 预测步：P = P + Q （无运动模型）
        P += Q;
        // 计算增益
        double K = P / (P + R);
        // 更新估计
        X += K * (measurement - X);
        // 更新误差协方差
        P = (1 - K) * P;
        return X;
    }

    void reset(double& value)
    {
        X = value;
        P = 1.0;
    }

    void setQ(double& newQ)
    {
        Q = newQ;
    }
    void setR(double& newR)
    {
        R = newR;
    }
    void setQandR(double& newQ, double& newR)
    {
        Q = newQ;
        R = newR;
    }
};
#endif