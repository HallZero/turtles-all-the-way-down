# turtles-all-the-way-down
ROS exercise

# Ambiente de desenvolvimento ROS
Robotic Operational System (ROS) é um conjunto de bibliotecas e ferramentas permite desenvolver códigos para uso na área da robótica.

## Passos
1. Instalação do Ubuntu: Foi necessária a intalação do WSL (Windows Subsystem for Linux) que permite a utilização de um ambiente GNU/Linux (Neste caso, o Ubuntu v22.04) diretamente pelo Windows.
2. Instalação dos pacotes do ROS2: Após as configurações necessárias, foi instalado os pacotes do ROS2. Neste caso, a versão Humble foi a utilizada.
`sudo apt update`
`sudo apt upgrade`
`sudo apt install ros-humble-desktop`
3. Funcionamento dos nós e testes com "turtlesim_node" e "turtle_teleop_key"

## Criação dos Scripts de para desenho dos Kanjis "菊池" (Kikuchi)
Inicialmente, a escolha para a representação dos Kanjis teve como finalidade forçar o aprendizado das funcionalidades e capacidades da biblioteca, além de adicionar uma característica pessoal e constituir a multidisciplinaridade de vários conceitos.

### O código
Nesse repositório, há dois arquivos nomeados `"drawing-turtle.py"` e `"test-turtle.py"`, que são, respectivamete, o arquivo da entrega efetiva e um arquivo para testar funcionalidades. Ambos são semelhantes, embora o arquivo de teste tenha funcionalidades não implementadas no arquivo principal.

Ambos os códigos contém uma classe `Turtlehub`, a qual a a criação de um nó com o namespace `'turtle-hub'` e alguns atributos e métodos:
- `'tp'`: Cria um cliente para o serviço `'/turtle1/teleport_absolute'` do tipo `TeleportAbsolute`
- `'pen'`: Cria um cliente para o serviço `'/turtle1/set_pen'` do tipo `SetPen`
- `'spawn_'`: Cria um cliente para o serviço `'/spawn'` do tipo `Spawn`
- `'draw_'`: Cria um publisher para o tópico `'turtle1/cmd_vel'`, utilizando uma mensagem do tipo `Twist`
- `'status'`: Variável para controle do estado da Caneta
- `'togglepen'`: método que cria uma request do tipo SetPen. Toma como parâmetro a variável de estado `'status'` (*1)
- `'teleport'`: método que cria uma request do tipo TeleportAbsolut. Toma como parâmetros três variáveis (x,y,theta), que representam a orientação do nó no plano
- `'draw'`: método que cria uma mensagem do tipo Twist, utilizando como parâmetros dois valores que representam, respectivamente, a velocidade linear e angular do nó. Após isso, o método publica a mensagem no tópico descrito acima.

Na função main, a comunicação ROS é inicializada e cria-se o nó turtle-hub do tipo TurtleHub e então, começa um laço de repetição que tem como propósito:
1. Desligar a caneta
2. Teleportar o nó para o começo do traço do Kanji, cada um definido por uma posição no array `stroke`
3. Ligar a caneta
4. Desenhar sobre o plano com a velocidade linear e angular definidas
6. Esperar 1 segundo
Por fim, o nó controlador da tartaruga a direciona para a posição (0,0) do plano, é destruído e a comunicação é finalizada.

---

### Problemas encontrados:
- Em algumas execuções, o código fica travado em `rclpy.spin_until_future_complete(turtle_hub, pen)`. Devido à natureza assíncrona da chamada e a inexperiência do uso, ainda não há entendimento completo do porquê. :confused:
- Ainda não há implementação eficiente dos Design Patterns para este exercício. Espera-se que futuramente, haja uma melhora na construção de códigos, especialmente os que utilizam o paradigma de orientação a objeto. 
