# Guia: Gerar URDF a partir do Onshape com `onshape-to-robot`

Este guia resume o fluxo usado para gerar um arquivo URDF a partir de um Assembly do Onshape usando o pacote `onshape-to-robot`.

O objetivo aqui é apenas gerar o URDF e os meshes a partir do Onshape. Este guia não cobre visualização no RViz, integração com ROS 2, Gazebo, Isaac Sim ou `ros2_control`.

---

## 1. Estrutura inicial recomendada

Entre na pasta onde você quer manter os scripts de exportação:

```bash
cd ~/Workspaces/f450_ws/src/f450_uav/f450_description/scripts
```

A estrutura usada foi:

```text
scripts/
├── f450_robot/
│   └── config.json
├── requirements.txt
├── setup.sh
└── .venv/
```

A pasta `f450_robot/` é a pasta passada como argumento para o `onshape-to-robot`.

---

## 2. Criar o `requirements.txt`

Crie o arquivo:

```bash
nano requirements.txt
```

Conteúdo:

```txt
numpy<2.0
transforms3d
onshape-to-robot
```

O `numpy<2.0` é importante porque algumas versões do `transforms3d` ainda usam funções removidas no NumPy 2.x.

---

## 3. Criar o `setup.sh`

Crie o arquivo:

```bash
nano setup.sh
```

Conteúdo:

```bash
#!/bin/bash

set -e

echo "========================================="
echo " Creating Python virtual environment"
echo "========================================="

python3 -m venv .venv

echo "========================================="
echo " Activating virtual environment"
echo "========================================="

source .venv/bin/activate

echo "========================================="
echo " Upgrading pip"
echo "========================================="

pip install --upgrade pip

echo "========================================="
echo " Installing requirements"
echo "========================================="

pip install -r requirements.txt

echo "========================================="
echo " Installation finished"
echo "========================================="

echo ""
echo "To activate the environment:"
echo "source .venv/bin/activate"
echo ""

echo "Testing installation..."
onshape-to-robot --help
```

Dê permissão de execução:

```bash
chmod +x setup.sh
```

Execute:

```bash
./setup.sh
```

---

## 4. Ativar o ambiente virtual

Sempre que for usar o `onshape-to-robot`, ative o ambiente:

```bash
source .venv/bin/activate
```

O terminal deve ficar parecido com:

```text
(.venv) mendes@mendes-laptop:~/Workspaces/...
```

Teste:

```bash
onshape-to-robot --help
```

Se aparecer a ajuda do comando, a instalação está funcionando.

---

## 5. Criar API Key no Onshape

No Onshape, acesse a página de API Keys no Developer Portal:

```text
https://dev-portal.onshape.com/keys
```

Crie uma nova API Key.

Permissão mínima recomendada:

```text
Documents: Read
```

Depois de criar, o Onshape mostrará:

```text
Access Key
Secret Key
```

A Secret Key aparece apenas uma vez. Salve com cuidado.

> Observação de segurança: se você já expôs sua Secret Key em algum chat, repositório, print ou log, revogue essa chave no Onshape e gere uma nova.

---

## 6. Configurar variáveis no `.bashrc`

A versão usada do `onshape-to-robot` espera as credenciais como variáveis de ambiente.

Abra o `.bashrc`:

```bash
nano ~/.bashrc
```

Adicione no final do arquivo:

```bash
export ONSHAPE_API=https://cad.onshape.com
export ONSHAPE_ACCESS_KEY=SUA_ACCESS_KEY_AQUI
export ONSHAPE_SECRET_KEY=SUA_SECRET_KEY_AQUI
```

Exemplo com placeholders:

```bash
export ONSHAPE_API=https://cad.onshape.com
export ONSHAPE_ACCESS_KEY=on_xxxxxxxxxxxxxxxxxxxxxxxx
export ONSHAPE_SECRET_KEY=xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
```

Salve:

```text
CTRL + O
ENTER
CTRL + X
```

Recarregue o `.bashrc`:

```bash
source ~/.bashrc
```

Confirme se a variável foi carregada:

```bash
echo $ONSHAPE_ACCESS_KEY
```

Deve aparecer sua Access Key.

---

## 7. Obter o link correto do Onshape

Abra o documento compartilhado no Onshape.

É importante abrir o **Assembly principal**, não o Part Studio.

O link deve ser parecido com:

```text
https://cad.onshape.com/documents/XXXXX/w/YYYY/e/ZZZZZ
```

ou:

```text
https://cad.onshape.com/documents/XXXXX/m/YYYY/e/ZZZZZ
```

Esse link será usado no `config.json`.

---

## 8. Preparar o Assembly no Onshape

Para o URDF ser gerado corretamente, o Assembly precisa formar uma árvore única.

Regras importantes:

1. Deve existir apenas uma raiz/base principal.
2. Para um drone F450 rígido, normalmente todos os componentes devem estar conectados por `Fastened Mate`.
3. Se houver hélices com `Revolute Mate`, o `onshape-to-robot` pode interpretar múltiplas bases se a cadeia não estiver corretamente conectada.
4. Para gerar um URDF simples de UAV rígido, recomenda-se deixar as hélices também como `Fastened Mate` inicialmente.

Resultado esperado no terminal:

```text
Found 1 root node
Found total 0 degrees of freedom
```

Para um drone rígido, `0 degrees of freedom` está correto.

Se aparecer:

```text
WARNING: Multiple base links detected, which is not supported by URDF.
Only the first base link will be considered.
```

isso indica que existem peças ou subconjuntos desconectados no Assembly.

---

## 9. Criar a pasta do robô

Na pasta `scripts/`:

```bash
mkdir -p f450_robot
```

---

## 10. Criar o `config.json`

Crie o arquivo:

```bash
nano f450_robot/config.json
```

Conteúdo recomendado:

```json
{
  "url": "COLE_AQUI_A_URL_DO_ASSEMBLY_DO_ONSHAPE",

  "output_format": "urdf",

  "robot_name": "f450_uav",

  "package_name": "f450_description",

  "mesh_format": "dae"
}
```

Exemplo:

```json
{
  "url": "https://cad.onshape.com/documents/XXXXX/w/YYYY/e/ZZZZZ",

  "output_format": "urdf",

  "robot_name": "f450_uav",

  "package_name": "f450_description",

  "mesh_format": "dae"
}
```

Também é possível usar:

```json
"mesh_format": "stl"
```

Mas esse campo **não significa usar um STL local**. Ele apenas define em qual formato o `onshape-to-robot` exportará os meshes a partir do Onshape.

Opções comuns:

```text
dae
stl
```

Recomendação geral:

```json
"mesh_format": "dae"
```

---

## 11. Gerar o URDF

Estando na pasta `scripts/`, execute:

```bash
onshape-to-robot f450_robot
```

Atenção: nesta versão, o comando recebe a **pasta do robô**, não o arquivo `config.json` diretamente.

Correto:

```bash
onshape-to-robot f450_robot
```

Incorreto para esta versão:

```bash
onshape-to-robot f450_robot/config.json
```

---

## 12. Resultado esperado

Depois da execução, a pasta deve ficar parecida com:

```text
f450_robot/
├── config.json
├── robot.urdf
├── robot.pkl
└── meshes/
    ├── part_1.dae
    ├── part_2.dae
    └── ...
```

Verifique com:

```bash
tree f450_robot
```

O arquivo principal gerado é:

```text
f450_robot/robot.urdf
```

---

## 13. Erros comuns

### Erro: `No Onshape API access key are set`

Causa: as variáveis de ambiente não foram configuradas ou não foram carregadas.

Solução:

```bash
source ~/.bashrc
```

Depois confira:

```bash
echo $ONSHAPE_ACCESS_KEY
```

Se não aparecer nada, revise o `.bashrc`.

---

### Erro relacionado ao NumPy 2.x

Exemplo:

```text
AttributeError: `np.maximum_sctype` was removed in the NumPy 2.0 release
```

Solução: usar `numpy<2.0` no `requirements.txt` e reinstalar no ambiente virtual.

```bash
source .venv/bin/activate
pip uninstall -y numpy transforms3d onshape-to-robot
pip install -r requirements.txt
```

---

### Aviso: múltiplas bases

Exemplo:

```text
WARNING: Multiple base links detected, which is not supported by URDF.
Only the first base link will be considered.
```

Causa: há partes desconectadas no Assembly.

Solução no Onshape:

1. Escolha uma peça principal como base, por exemplo `Bottom plate`.
2. Fixe essa peça no Assembly.
3. Conecte todas as demais peças à base por uma cadeia única de mates.
4. Para UAV rígido, use `Fastened Mate` para arms, motores, hélices, placas, bateria, ESCs e eletrônicos.
5. Salve o Assembly.
6. Execute novamente:

```bash
onshape-to-robot f450_robot
```

---

## 14. Observação sobre STL local

Um arquivo STL local não é usado diretamente pelo `onshape-to-robot`.

O campo:

```json
"mesh_format": "stl"
```

significa apenas que o exportador vai baixar os meshes do Onshape em formato STL.

Para o `onshape-to-robot`, o dado principal é o Assembly do Onshape, porque ele contém:

- peças;
- posições;
- mates;
- hierarquia;
- possíveis juntas;
- propriedades físicas.

Um STL isolado contém apenas geometria triangular e não contém árvore cinemática.

---

## 15. Sequência resumida de comandos

```bash
cd ~/Workspaces/f450_ws/src/f450_uav/f450_description/scripts

nano requirements.txt
nano setup.sh
chmod +x setup.sh
./setup.sh

nano ~/.bashrc
source ~/.bashrc

mkdir -p f450_robot
nano f450_robot/config.json

source .venv/bin/activate
onshape-to-robot f450_robot

tree f450_robot
```

---

## 16. Resultado final

Ao final do processo, o URDF estará em:

```text
f450_robot/robot.urdf
```

E os meshes estarão em:

```text
f450_robot/meshes/
```