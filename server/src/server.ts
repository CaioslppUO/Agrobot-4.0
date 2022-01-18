import express from 'express';
import bodyParser from 'body-parser';
import serverHttp from 'http';
import SocketIO from 'socket.io';
import Control from './types/controlInterface';
import { controlDefaultParams } from './types/controlInterface';
import AutoModeParams from './types/autoModeParams';
import { AutoModeDefaultParams } from './types/autoModeParams';
import { Wheel_adjustment, wheel_adjustmentDefaultParams } from "./types/controlInterface";

const app = express();
const http = serverHttp.createServer(app);
const io = SocketIO(http);

app.use(bodyParser.urlencoded({ extended: false }));

let control: Control = controlDefaultParams;
let autoModeData: AutoModeParams = AutoModeDefaultParams;
let wheel_adjustment: Wheel_adjustment = wheel_adjustmentDefaultParams;


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
    control = data;
  });

  /**
   * EVENTO DE ATUALIZAÇÃO DO ESTADO DOS PARAMS AUTO MODE
   * Evento ativado pelo app, alterando a instancia "autoModeData"
   */
  socket.on('auto_mode_params_update', (data) => {
    autoModeData = data;
  });

  /**
   * EVENTO DE ATUALIZAÇÃO DO ESTADO DO AJUSTE MANUAL DAS RODAS
   * Evento ativado pelo app, alterando a instancia "wheel_adjustment"
   */
  socket.on('manual_wheel_adjustment_update', (data) => {
    wheel_adjustment = data;
  })

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
 * ROTA DE DO AJUSTE DAS RODAS
 * Retorna o estado atual do ajuste das rodas
 */
app.get("/manual_wheel_adjustment", (req, res) => {
  return res.json(wheel_adjustment);
})

/**
 * INICIALIZACAO DO SERVIDOR
 */
http.listen(3000, () => {
  console.log('listening on *:3000');
});
