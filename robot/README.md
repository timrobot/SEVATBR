<h2>ids for the arduino</h2>
1: wheels
2: arm
3: claw

<h2>format</h2>
arduino:
- wheels:
  - input:
    - [%d %d] (left, right)
  - output:
    - [1 %d %d %d %d] (encoder left, encoder right, ultrasonic back-left, ultrasonic back-right)
- arm:
  - input:
    - [%d %d] (base, pivot)
  - output:
    - [2 %d %d %d %d] (base deg, pivot deg, ultrasonic front-left, ultrasonic front-right)
- claw:
  - input:
    - [%d %d] (rotate, pincer)
  - output:
    - [3] (nothing)

base:
- wheels:
  - input:
    - [1 %d %d %d %d] (encoder left, encoder right, ultrasonic back-left, ultrasonic back-right)
  - output:
    - [%d %d] (left, right)
- arm:
  - input:
    - [2 %d %d %d %d] (base deg, pivot deg, ultrasonic front-left, ultrasonic front-right)
  - output:
    - [%d %d] (base, pivot)
- claw:
  - input:
    - [3] (nothing)
  - output:
    - [%d %d] (rotate, pincer)
