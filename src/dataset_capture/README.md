# dataset_capture

Pacote ROS2 (Humble) para capturar imagens da camera do robo e montar
um dataset de treino (ex.: YOLO).

## Estrutura

- `camera_publisher.py` — abre a camera e publica continuamente em
  `sensor_msgs/Image` (topico padrao `/dataset_capture/image_raw`).
  Nao salva nada em disco, so publica o que a camera esta vendo.
- `image_saver.py` — assina o topico da camera e SALVA os frames em
  uma pasta local. As fotos so comecam a ser tiradas quando este no
  e executado.

## Instalacao

Copie a pasta `dataset_capture/` para dentro do `src/` do seu
workspace (`RoboFEI-HT_2023_SOFTWARE`), ao lado dos outros pacotes
(`vision_pkg`, `motors_pkg`, etc.), depois:

```bash
cd ~/RoboFEI-HT_2023_SOFTWARE   # ajuste para o caminho real do workspace
colcon build --symlink-install --packages-select dataset_capture
source install/setup.bash
```

## Uso

**Antes de comecar**: pare o no de visao (`detect.py` / `image_node`, do
`vision_pkg`) se ele estiver rodando. So um processo por vez consegue
abrir `/dev/camera`.

1) Suba o publisher da camera (pode ficar ligado o tempo todo,
   nao grava nada sozinho):

```bash
ros2 run dataset_capture camera_publisher --ros-args -p device:=/dev/camera
```

Se preferir usar o indice numerico (ex: `/dev/video0`), pode passar
`-p device:=0`.

2) Quando estiver pronto para tirar as fotos, suba o saver — é so
   nesse momento que a captura comeca de fato:

```bash
ros2 run dataset_capture image_saver --ros-args \
    -p output_dir:=/home/robofei/dataset_bola \
    -p num_images:=1000 \
    -p mode:=auto \
    -p interval:=0.5
```

O no encerra sozinho assim que atingir as 1000 fotos.

### Modo manual (recomendado para variar poses/angulos)

Se quiser controlar exatamente quando cada foto e tirada (ex.: indo
mudando a posicao do robo/bola entre uma foto e outra), use o modo
manual: cada `ENTER` no terminal onde o no esta rodando tira uma foto.

```bash
ros2 run dataset_capture image_saver --ros-args \
    -p output_dir:=/home/robofei/dataset_bola \
    -p num_images:=1000 \
    -p mode:=manual
```

### Subindo os dois juntos via launch

```bash
ros2 launch dataset_capture dataset_capture.launch.py \
    output_dir:=/home/robofei/dataset_bola \
    num_images:=1000 \
    mode:=auto
```

## Observacoes

- Os arquivos sao salvos como `img_00000.jpg`, `img_00001.jpg`, ...
  Se a pasta ja tiver imagens com o mesmo prefixo, a contagem
  continua de onde parou (nao sobrescreve fotos antigas).
- Ajuste `frame_width`/`frame_height` no `camera_publisher` se quiser
  fixar a resolucao de captura.
- `image_format` aceita `jpg` ou `png`.
