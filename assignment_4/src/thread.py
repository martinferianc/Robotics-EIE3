import threading

class Poller():

   def __init__(self,t,target):
      self.t=t
      self.target = target
      self.thread = threading.Timer(self.t,self.handle_function)

   def handle_function(self):
      self.target()
      self.thread = threading.Timer(self.t,self.handle_function)
      self.thread.start()

   def start(self):
      self.thread.start()

   def stop(self):
      self.thread.cancel()

#t = Poller(5,printer)
#t.start()
#t.stop()
