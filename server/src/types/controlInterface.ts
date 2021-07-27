interface Control {
  speed: number;
  steer: number;
  limit: number;
  module: boolean;
  autoMode: boolean;
  power: boolean;
}

export const controlDefaultParams: Control = {
  speed: 0,
  steer: 0,
  limit: 0,
  module: false,
  autoMode: false,
  power: false,
};

export default Control;
