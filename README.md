# c400 AreaDetector

## Information

- This IOC allows the control and acquisition of the c400 (Four-channel Pulse Counting Detector Controller), this IOC was based on ADDrive and allows the use of native AreaDetector plugins.

## Image Mode

- With this IOC it is possible to make acquisitions using Buffered or Unbuffered.

- When Unbuffering is configured (```$(P)$cam1:ImageMode```) it is possible to perform acquisitions and monitoring in real time, but it is necessary to pay attention to the integration time, much shorter times can result in loss of frame (communication rate limit), it is possible to monitor image loss by the UniqueId of each frame (```$(P)$image1:UniqueId_RBV```). In this mode, it is also possible to monitor the ENCODER in real time.

- When Buffering is configured (```$(P)$cam1:ImageMode```) it is possible to make acquisitions with integration time in ns, but the images will be updated every 400 acquisitions. With Buffer mode, it is not allowed to define the number of images >65536 (```$(P)cam1:NumImages```), the maximum buffer of the equipment. **NOTE:** The c400 does not have a circular buffer.

## Epics in Docker

- This IOC supports epics-in-docker (https://github.com/cnpem/epics-in-docker).

## Developer

Developer: Guilherme Rodrigues de Lima
Email: guilherme.lima@lnls.br
Company: CNPEM/Sirius - Brazil
Date: 10/03/2023

Based on the IOC asynDrive c400 by Hugo Henrique Valim.
