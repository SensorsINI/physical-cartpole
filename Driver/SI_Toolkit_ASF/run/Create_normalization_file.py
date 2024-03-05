import os
import yaml
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"  # TF: If uncommented, only uses CPU
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "1"
from SI_Toolkit.load_and_normalize import calculate_normalization_info

config = yaml.load(open(os.path.join('SI_Toolkit_ASF', 'config_training.yml'), 'r'), Loader=yaml.FullLoader)

calculate_normalization_info(config=config)
