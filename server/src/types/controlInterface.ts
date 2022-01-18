interface Control {
  speed: number;
  steer: number;
  limit: number;
  module: boolean;
  autoMode: boolean;
  power: boolean;
}

export interface Wheel_adjustment {
  wheel: number;
  direction: number;
};

export const wheel_adjustmentDefaultParams = {
  wheel: 0,
  direction: 0,
};

export const controlDefaultParams: Control = {
  speed: 0,
  steer: 0,
  limit: 0,
  module: false,
  autoMode: false,
  power: false,
};

export default Control;
