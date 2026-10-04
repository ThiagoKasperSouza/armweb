# armweb — simulação de braço robótico com ROS 2 + OpenUSD, headless e sem GPU

<img width="910" height="404" alt="image" src="https://github.com/user-attachments/assets/f3f9bbee-3ae3-4695-999e-9909489988e1" />


Infraestrutura completa em Docker/WSL para simular um braço robótico de 6 GDL,
exportar o modelo em **OpenUSD** e visualizar tudo **pelo navegador**.

Não requer GPU: a simulação é cinemática (publica `JointState` + TF) e a
visualização usa o Foxglove Web via WebSocket — nada precisa de OpenGL no servidor.

---

## Começando

```bash
cd /home/thiag/armweb
make up
```

Depois abra **https://app.foxglove.dev** no navegador do Windows e conecte em:

```
ws://localhost:8765
```

Comandos úteis:

```bash
make help      # lista todos os alvos
make status    # topics, nós, joint states, frames TF
make cmd       # envia uma pose de exemplo (o braço se move)
make usd       # regenera e valida data/usd/arm.usda
make record    # grava /joint_states -> data/usd/arm_anim.usda (animado)
make gltf      # exporta .glb para o viewer three.js
make web       # serve o viewer three.js em :8080
make logs      # logs do container
make down      # derruba o stack
```

### Três formas de visualizar

| Alvo | Onde | Como |
|---|---|---|
| **Tempo real (ROS)** | [app.foxglove.dev](https://app.foxglove.dev) | `make up` → conectar `ws://localhost:8765` |
| **Modelo 3D web (glTF)** | `web/viewer.html` | `make gltf` → `make web` → abrir `http://localhost:8080/web/viewer.html` |
| **Asset OpenUSD** | `data/usd/*.usda` | abrir no Omniverse/usdview (copie via `\\wsl$\Ubuntu\...`) |

---

## Arquitetura

```
┌───────────── WSL2 (Ubuntu 26.04) ─────────────────────────────────┐
│                                                                     │
│  host:8765 ──► foxglove_bridge ──┐                                 │
│                                  │   ROS 2 DDS (FastDDS)            │
│  ┌───────────────────────────────▼───────────────────────────────┐  │
│  │ container armweb_sim  (ros:humble-ros-base + usd-core)       │  │
│  │                                                                │  │
│  │  robot_state_publisher   URDF → /tf, /tf_static                │  │
│  │  arm_sim_node            /joint_states @ 50 Hz                │  │
│  │  export_usd.py           URDF → arm.usda  (OpenUSD/pxr)        │  │
│  └────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
             │                                            │
     app.foxglove.dev (browser)              data/usd/arm.usda (OpenUSD)
```

## Estrutura

```
armweb/
├── Dockerfile              ros:humble-ros-base + ros2_control + foxglove + usd-core
├── docker-compose.yml      serviço `sim`, porta 8765
├── Makefile                interface de operações
├── .env                    ROS_DOMAIN_ID, portas
├── scripts/
│   ├── status.sh           diagnóstico ao vivo
│   ├── send_cmd.sh         envia pose para o braço
│   ├── export_usd.sh       gera e valida o USD
│   └── clean_ws.sh         limpa artefatos de build
├── data/usd/               saída OpenUSD (arm.usda, arm_anim.usda)
├── data/gltf/              saída glTF (.glb) para o viewer web
├── web/viewer.html         viewer three.js (GLTFLoader + AnimationMixer)
└── ws/
    ├── assets/urdf/        robot.urdf (do prepare_arm.sh) + demo_arm.urdf
    ├── scripts/            entrypoint.sh, healthcheck.sh
    ├── src/armweb_sim/     pacote ROS 2 (ament_python)
    │   ├── armweb_sim/arm_sim_node.py   simulador cinemático
    │   └── launch/sim.launch.py          rsp + sim + foxglove
    └── tools/
        ├── gen_urdf.py          gera o URDF procedural
        ├── export_usd.py        URDF → OpenUSD (pose estática)
        ├── record_usd_animation.py  /joint_states → OpenUSD animado
        ├── export_gltf.py       URDF → glTF 2.0 (.glb), com animação
        ├── reopen_check.py      valida o .usda
        ├── verify_anim.py       confere time samples do .usda
        ├── validate_glb.py      valida estrutura/chunks do .glb
        └── verify_gltf_scene.py percorre a hierarquia: raízes, bbox,
                                 orientação Y-up e animação
```

---

## Controlando o braço

O nó `arm_sim_node` assina `std_msgs/Float64MultiArray`:

| tópico | tipo | descrição |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | estado das 7 juntas @ 50 Hz |
| `/arm_sim/joint_positions` | `Float64MultiArray` | eco das posições |
| `/arm_sim/arm/command` | `Float64MultiArray` | **alvo** (in) |
| `/tf`, `/tf_static` | `tf2_msgs` | árvore cinemática |

Ordem das juntas:
`shoulder_pan, elbow_pitch, forearm_roll, wrist_1, wrist_2, wrist_3, gripper`

```bash
./scripts/send_cmd.sh 1.0 -0.5 0.3 1.2 0.0 0.5 0.04
```

## OpenUSD

`make usd` gera `data/usd/arm.usda` (54 prims, materiais `UsdPreviewSurface`
vinculados, Z-up, metros).

`make record` grava a animação: assina `/joint_states`, constrói a árvore
cinemática a partir do URDF e autora **um time sample por frame** em cada junta.

```bash
make record                      # 5s @ 20 Hz
make record DURATION=10 RATE=30  # customizado
```

Resultado (`data/usd/arm_anim.usda`):

```
timeCodesPerSecond: 20.0
joint_shoulder_pan_joint.orient: 51 samples, t=[0.0..50.0], changes=True
joint_gripper_joint.translate:   51 samples, t=[0.0..50.0], changes=True
total time samples: 357
```

Juntas revolute usam `xformOp:orient` (quaternion exato para qualquer eixo);
a prismática usa `xformOp:translate`. Abra no usdview/Omniverse e a timeline
já está configurada para 20 fps.

> **Dica:** para a gravação conter movimento, o braço precisa se mover. Publique
> comandos durante a janela de gravação (outro terminal: `make cmd`).

## glTF / three.js

`make gltf` gera três `.glb`:

| Arquivo | Conteúdo |
|---|---|
| `arm_rest.glb` | pose de repouso |
| `arm_posed.glb` | pose `1.0,-0.5,0.3,1.2,0.0,0.5,0.04` |
| `arm_animated.glb` | **com a animação** gravada em `make record` |

```bash
make gltf
make web
# abrir http://localhost:8080/web/viewer.html
```

O viewer tem seletor de modelo, play/pause, wireframe, recentralização
automática e seletor de arquivo local (arraste um `.glb` seu).

Como `usd-core` **não tem `GLTFWriter`**, o `export_gltf.py` é próprio: tesselamos
as primitivas (caixa/cilindro/esfera) em meshes reais, escrevemos o buffer
binário GLB à mão e, quando há animação, convertemos os time samples do USD em
*animation channels* glTF (`rotation`/`translation`, interpolação LINEAR).

Para inspecionar:

```bash
# dentro do container
docker compose exec sim python3 -c "
from pxr import Usd; s=Usd.Stage.Open('/ws/data/usd/arm.usda')
print(len(list(s.Traverse())), 'prims')"
```

Copie para o Windows para abrir no Omniverse / usdview local:
`\\wsl$\Ubuntu\home\thiag\armweb\data\usd\arm.usda`

---

## Robôs disponíveis (malhas reais, open source)

Não precisamos modelar nada: o projeto usa descrições ROS oficiais, instaladas
no container e prontas para exportar.

| Comando | Robô | Licença | Juntas | Malhas |
|---|---|---|---|---|
| `make arm ARM=demo` | braço procedural (padrão antigo) | — | 7 | primitivas |
| `make arm ARM=fr3` | **Franka Emika FR3** | Apache-2.0 | 13 (9 atuadas) | 8 DAE (~5 MB) |
| `make arm ARM=ur5` | **Universal Robots UR5** | BSD-3 | 6+ | STL + DAE |

```bash
make arm ARM=fr3   # resolve o xacro -> URDF plano
make up            # reinicia a simulação já com o FR3
make gltf          # gera fr3_rest.glb / fr3_posed.glb / fr3_animated.glb
make web           # http://localhost:8080/web/viewer.html
```

Os pacotes vêm do próprio repositório ROS 2 (`packages.ros.org/ros2`), então
não há download externo nem implicação de licença extra:

```dockerfile
RUN apt-get install -y ros-humble-franka-description ros-humble-ur-description
```

`prepare_arm.sh` resolve o `.urdf.xacro` com `xacro` e grava
`ws/assets/urdf/robot.urdf`; o simulador ROS lê os nomes das juntas desse
arquivo (`arm_sim: 13 joints from robot.urdf`), então a mesma stack anima
qualquer um dos braços sem configuração.

### Carregamento de malhas

`ws/tools/load_mesh.py` resolve URIs `package://` e lê:

* **STL / OBJ / PLY** via `trimesh`
* **DAE / COLLADA** via `pycollada` diretamente

> **Por que não usar `trimesh` para DAE?** Com numpy 2.x o `trimesh` 5.x falha
> nesses arquivos com *"only length-1 arrays can be converted to Python
> scalars"*. Ler o COLLADA diretamente contorna o problema. Também é preciso
> tratar o rank variável do array de índices (`(N,3)`, `(N,3,2)` ou `(N,2)`) e
> os normais por canto via `normal_index`.

### Escala automática das malhas

O `franka_description` modela em **milímetros** e **não declara `<scale>`**, o
que produzia um braço de 300 m. `export_gltf.py` detecta isso comparando a
extensão bruta da malha com o alcance cinemático da árvore (em metros) e aplica
a escala decade correta automaticamente:

```
[export_gltf] scale check: reach=1.539 m, raw mesh extent=312.3 (ratio 203x)
              -> applying scale 0.00316 (scaled extent 0.99 m)
```

## Notas importantes (decisões técnicas)

* **Sem GPU, sem engine de física.** Gazebo sem GPU gasta CPU no render loop e o
  resultado visual no Foxglove é idêntico. A simulação é cinemática; para
  dinâmica/torque real, use MuJoCo ou `ros2_control` com `gz_ros2_control`.
* **`usd-core` do PyPI não inclui `UsdUrdfParser` nem `GLTFWriter`.** Por isso
  `export_usd.py` e `export_gltf.py` são conversores próprios sobre a API `pxr`
  (e `struct`/`json` da stdlib no caso do glTF).
* **glTF é Y-up, URDF/USD são Z-up.** O exportador cria um nó raiz `zup_to_yup`
  com rotação de **−90° em X**, que mapeia `+Z → +Y`. Atenção: `+90°` inverteria
  o modelo (foi exatamente o bug da primeira versão). O viewer **não** rotaciona
  nada — a conversão acontece na origem, então qualquer consumidor glTF já
  recebe o modelo de pé.
* **Só as raízes reais entram em `scenes[0].nodes`.** Listar todos os nós ali
  achata a hierarquia e faz o braço aparecer desmontado. A cadeia correta é
  `link → joint → link`, com o nó da junta pendurado no link **pai**.
* **Ordem de quaternion:** USD expõe `(w, x, y, z)`; glTF exige `(x, y, z, w)`.
  Gravar direto produz `(w, 0, 0, z)`, que o three.js lê como `x = w` — a junta
  fica parada e o modelo parece "de cabeça para baixo" ou torto.
* **`AddOrientOp()` exige `Gf.Quatf`, não `Gf.Quatd`** — erro de tipo
  `expected 'GfQuatf', got 'GfQuatd'`. Por isso as juntas rotativas usam `Quatf`.
* **Esta build do `pxr` não tem `Xform.GetOrientAttr()`/`GetTranslateAttr()`.**
  Leia os atributos via `GetOrderedXformOps()` + `op.GetAttr()`.
* **A timeline precisa ser configurada antes de autorar** os time samples,
  senão o stage fica com 24 fps (default) em vez da taxa pedida.
* **`set -u` é incompatível com `setup.bash` do ROS 2** — os scripts usam
  `set -eo pipefail` e definem `AMENT_TRACE_SETUP_FILES=0`.
* **O offset de `<visual>` não move o frame do link.** A origem de uma junta é
  definida no frame do link **pai**; somar o offset visual ao nó do link
  duplica o valor e desloca cada junta filha — é o que fazia os segmentos
  parecerem soltos. O offset vai num nó filho dedicado (`<link>_visual`).
* **O simulador lê os nomes das juntas do URDF** (`ARMWEB_URDF`), então a mesma
  stack anima o braço demo, o FR3 ou o UR5 sem reconfiguração.
* **Malhas em COLLADA têm shape de índice variando** (`(N,3)`, `(N,3,2)`, `(N,2)`)
  e normais indexados por canto. Tratar isso explicitamente foi necessário para
  carregar os meshes do Franka.
* **`pycollada` é o nome do pacote; o módulo importado é `collada`.**
* **Arquivos editados no Windows chegam com CRLF** e quebram o bash
  (`set: pipefail\r: invalid option name`). O `sync.sh` normaliza para LF e
  valida; se você editar arquivos direto no WSL, use LF.
* **`ament_python` vs `ament_cmake`:** o pacote tem console script Python, logo
  é `ament_python`; o executável fica em `<prefix>/bin`, e o launch o invoca
  como `python3 -m armweb_sim.arm_sim_node`.
* **`robot_description`** precisa receber o *conteúdo* XML do URDF (lido em
  tempo de launch via `OpaqueFunction`), não o caminho — senão o parser C++
  falha com "Error document empty".
* **Artefatos do colcon são root-owned** (build dentro do container);
  `scripts/clean_ws.sh` remove como root dentro de um container.

## Acesso remoto

Para expor a um servidor externo, prefira túnel/VPN a abrir a porta:

```bash
# do lado do cliente
ssh -N -L 8765:localhost:8765 usuario@servidor
```

Depois conecte `ws://localhost:8765` no Foxglove.
