import express from 'express';
import bodyParser from 'body-parser';
import serverHttp from 'http';
import SocketIO from 'socket.io';
import Control from './types/controlInterface';
import { controlDefaultParams } from './types/controlInterface';
import AutoModeParams from './types/autoModeParams';
import { AutoModeDefaultParams } from './types/autoModeParams';

const app = express();
const http = serverHttp.createServer(app);
const io = SocketIO(http);

app.use(bodyParser.urlencoded({ extended: false }));

let control: Control = controlDefaultParams;
let autoModeData: AutoModeParams = AutoModeDefaultParams;


/**
 * SOCKET CONFIGURATION
 */
io.on('connection', (socket) => {
  console.log('A user connected');


  /**
   * EVENTO DE ATUALIZAÇÃO DO ESTADO DE CONTROLE
   * Evento ativado pelo app, alterando a instancia "control"
   */
  socket.on('control_update', (data) => {
    console.log('Control update: ', data);
    control = data;
  });

  /**
   * EVENTO DE ATUALIZAÇÃO DO ESTADO DOS PARAMS AUTO MODE
   * Evento ativado pelo app, alterando a instancia "autoModeData"
   */
  socket.on('auto_mode_params_update', (data) => {
    console.log('Auto mode params update: ', data);
    autoModeData = data;
  });

  /**
   * EVENTO DE DESCONEXÃO COM O APLICATIVO
   * Evento ativado quando ocorre a desconexão com a aplicativo,
   * atualizando todos os dados para os valos default
   */
  socket.on('disconnect', (data) => {
    console.log(`${socket.id} disconnected`);
    control = controlDefaultParams;
    autoModeData = AutoModeDefaultParams;
  });
});

/**
 * ROTA BASE
 * Retorna um titulo html caso o server esteja rodando normalmente
 */
app.get('/', (req, res) => {
  res.send('<h1>CoronaKiller server running</h1>');
});

/**
 * ROTA DE CONTROLE
 * Retorna o estado atual de controle
 */
app.get('/control', (req, res) => {
  return res.json(control);
});

/**
 * ROTA DE PARAMETROS DO MODE AUTOMATICO
 * Retorna o estado atual dos parametros do modo automatico
 */
app.get('/auto_mode_params', (req, res) => {
  return res.json(autoModeData);
});

/**
 * METODO PARA ENVIAR DADOS DO ROBO PARA O APLICATIVO
 */
app.post('/send_to_app', (req, res) => {
  const obj = req.body;
  io.emit('data_from_robot', req.body);
  return res.json({ message: 'Ready' });
});

/**
 * INICIALIZACAO DO SERVIDOR
 */
http.listen(3000, () => {
  console.log('listening on *:3000');
});
