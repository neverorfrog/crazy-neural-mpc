import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))


def log(message='', ros2_logger=None, ros2_logger_level='info'):
  if ros2_logger is None:
    print(message)
    return
  
  if ros2_logger_level == 'info': ros2_logger.info(message)
  elif ros2_logger_level == 'warn': ros2_logger.warn(message)
  elif ros2_logger_level == 'error': ros2_logger.error(message)
  elif ros2_logger_level == 'debug': ros2_logger.debug(message)
  else: ros2_logger.info(message)
  
  return
