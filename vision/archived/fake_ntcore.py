
class PubSubOptions:
  def __init__(self):
    pass

class FakeTopic:
  def __init__(self, name):
    self.name = name

  def publish(self, options):
    return self

  def set(self, value):
    pass

class NetworkTableInstance:
  instance = None

  def getDefault():
    if NetworkTableInstance.instance is None:
      NetworkTableInstance.instance = NetworkTableInstance()
    return NetworkTableInstance.instance
  
  def setServerTeam(self, team):
    pass

  def startClient4(self, name):
    pass

  def getFloatArrayTopic(self, name):
    return FakeTopic(name)
  
  def flush(self):
    pass
