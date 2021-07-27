interface AutoModeParams {
  limit: number;
  steer: number;
  speed: number;
  correctionsMovements: number;
  correctionFactor: number;
  detectDistance: number;
  moveTime: number;
  stopTime: number;
  module: boolean;
  autoMode: boolean;
}

export const AutoModeDefaultParams = {
  limit: 50,
  steer: -2,
  speed: -26,
  correctionsMovements: 5,
  correctionFactor: 15,
  detectDistance: 1.5,
  moveTime: 0,
  stopTime: 0,
  module: true,
  autoMode: false
};

export default AutoModeParams;
