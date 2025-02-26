#ifndef RC_INTERFACE_H
#define RC_INTERFACE_H

#include <Arduino.h>
#include <PWMServo.h>

/**
 * @brief ESC（電動スピードコントローラ）を制御するクラス
 * 
 * - 0〜100 の速度指令を入力として，PWM 出力を生成
 * - update() を定期的に呼び出し，台形制御でスピードを目標値へ近づける
 * - stop() で停止（速度0）
 */
class ESC {
public:
    /**
     * @brief コンストラクタ
     * @param pin ESC につないだピン番号
     * @param minPulse ESC の最小パルス幅(μs)
     * @param maxPulse ESC の最大パルス幅(μs)
     * @param accel 最大加速度(単位: [速度/秒]：例えば 100/s なら1秒で0→100まで加速可能)
     * 
     * ※ minPulse, maxPulse は一般的に 800, 2200(μs) など
     */
    ESC(int pin, int minPulse = 800, int maxPulse = 2200, float accel = 50.0f)
        : _pin(pin)
        , _minPulse(minPulse)
        , _maxPulse(maxPulse)
        , _accel(accel)
        , _currentSpeed(0.0f)
        , _targetSpeed(0.0f)
        , _lastUpdateTime(0)
    {
    }

    /**
     * @brief PWM出力を開始する．setup()等で呼び出す想定．
     */
    void begin() {
        _servo.attach(_pin);
        // 初期化的に最小スロットルを送る
        writeSpeedToESC(0.0f);
    }

    /**
     * @brief [0, 100] の範囲で速度（スロットル）を指示
     * @param speed 0〜100
     */
    void setSpeed(float speed) {
        if (speed < 0.0f) speed = 0.0f;
        if (speed > 100.0f) speed = 100.0f;
        _targetSpeed = speed;
    }

    /**
     * @brief 速度を0にする（停止）
     */
    void stop() {
        _targetSpeed = 0.0f;
    }

    /**
     * @brief 毎フレーム呼び出して，速度を台形制御で更新し，ESCへ出力する
     * 
     * loop() 内などで定期的(例: 毎10msなど)に呼び出してください．
     */
    void update() {
        unsigned long now = millis();
        float dt = (now - _lastUpdateTime) / 1000.0f;  // ミリ秒→秒
        if (_lastUpdateTime == 0) {
            dt = 0; // 初回呼び出し時は0扱い
        }
        _lastUpdateTime = now;

        // 台形制御（最大加速度 _accel を考慮して_currentSpeedをtargetSpeedへ近づける）
        float diff = _targetSpeed - _currentSpeed;
        float maxDelta = _accel * dt;  // この時間ステップで変化できる最大値

        if (diff > maxDelta) {
            // 加速しきれない
            _currentSpeed += maxDelta;
        } else if (diff < -maxDelta) {
            // 減速しきれない
            _currentSpeed -= maxDelta;
        } else {
            // 十分小さいので目標速度に追従
            _currentSpeed = _targetSpeed;
        }

        // 実際にESCへ書き込み
        writeSpeedToESC(_currentSpeed);
    }

private:
    PWMServo _servo;
    int _pin;
    int _minPulse;
    int _maxPulse;
    float _accel;         // 最大加速度 [速度/秒] (0〜100 の速度指令域)
    float _currentSpeed;  // 現在の速度指令 [0〜100]
    float _targetSpeed;   // 目標速度指令 [0〜100]
    unsigned long _lastUpdateTime;

    /**
     * @brief ESCクラス内部で実際にPWM出力する関数
     * @param speed 0〜100
     */
    void writeSpeedToESC(float speed) {
        // 0〜100 → パルス幅(_minPulse〜_maxPulse)へマッピング
        int pulse = map((int)(speed + 0.5f), 0, 100, _minPulse, _maxPulse);
        writeMicroseconds(pulse);
    }

    void writeMicroseconds(int pulse) {
        // pulseを0〜180度に変換
        int angle = map(pulse, _minPulse, _maxPulse, 0, 180);
        _servo.write(angle);
    }
};

/**
 * @brief RCサーボを制御するクラス
 * 
 * - setPosition() で -100〜100 の範囲で指定
 * - それを minAngle, centerAngle, maxAngle に基づいてマッピングし，実際の角度へ反映
 */
class RCServo {
public:
    /**
     * @brief コンストラクタ
     * @param pin サーボの接続ピン
     * @param minAngle サーボの最小角（実機に合わせて設定）
     * @param centerAngle サーボのセンタ位置角
     * @param maxAngle サーボの最大角
     */
    RCServo(int pin, int minAngle = 0, int centerAngle = 90, int maxAngle = 180)
        : _pin(pin)
        , _minAngle(minAngle)
        , _centerAngle(centerAngle)
        , _maxAngle(maxAngle)
    {
    }

    /**
     * @brief PWM出力開始（attach）．
     *        Arduinoのsetup()等で呼び出す想定．
     */
    void begin() {
        _servo.attach(_pin);
        // センターにしておくなど，必要に応じて
        setPosition(0);
    }

    /**
     * @brief サーボの位置を -100〜100 の範囲で指定
     * @param pos -100 〜 +100
     */
    void setPosition(int pos) {
        if (pos < -100) pos = -100;
        if (pos > 100)  pos = 100;

        // -100〜0〜100 を最小角, 中心角, 最大角にマッピング
        // pos=-100 => minAngle, pos=0 => centerAngle, pos=100 => maxAngle
        // まず -100〜0 を minAngle〜centerAngle
        //      0〜100 を centerAngle〜maxAngle でマッピング
        float angle = 0.0f;
        if (pos < 0) {
            // -100〜0
            angle = map(pos, -100, 0, _minAngle, _centerAngle);
        } else {
            // 0〜100
            angle = map(pos, 0, 100, _centerAngle, _maxAngle);
        }

        // 実際にサーボへ書き込み
        _servo.write((int)angle);
    }

private:
    PWMServo _servo;
    int _pin;
    int _minAngle;
    int _centerAngle;
    int _maxAngle;
};

#endif // RC_INTERFACE_H
