



class Train():
	def __init__(self, load_command, save_command):
                self.load_command = load_command
                self.save_command = save_command

                self.model = SAC(action_dim =len(self.action_high),state_dim=len(self.all_state),hidden_dim=256,buffer_size=1000000)
                self.model_loader()

        def done(self):
                self.episode_num += 1
                print('\ndone episode', self.episode_num, 'total_reward is :', self.ep_reward[-1])
                self.reset()

        def train(self, done):
                if len(self.model.replay_buffer) > self.batch_size:
                        self.model.soft_q_update(self.batch_size, done, value_lr  = 0.5e-4, q_lr = 0.5e-4, policy_lr = 0.5e-4)


        def buffer_push(self, done):
                if self.last_state is not None and self.last_action is not None:
                self.model.replay_buffer.push(self.last_state, self.last_action, self.segmentReward[-1], self.all_state, done)

        def model_saver(self,model_path):
                if self.save_command=="on":
                torch.save(self.model.value_net.state_dict(), model_path+"value.dat")
                torch.save(self.model.target_value_net.state_dict(), model_path+"target.dat")
                torch.save(self.model.q_net.state_dict(), model_path+"qvalue.dat")
                torch.save(self.model.policy_net.state_dict(), model_path+"policy.dat")


        def model_loader(self,model_path):
                path = model_path
                if os.path.exists(model_path+"policy.dat") and self.load_command=="on":
                        self.model.value_net.load_state_dict(torch.load(model_path+"value.dat"))
                        self.model.value_net.eval()
                        self.model.target_value_net.load_state_dict(torch.load(model_path+"target.dat"))
                        self.model.target_value_net.eval()
                        self.model.q_net.load_state_dict(torch.load(model_path+"qvalue.dat"))
                        self.model.q_net.eval()
                        self.model.policy_net.load_state_dict(torch.load(model_path+"policy.dat"))
                        self.model.policy_net.eval()
                        self.model.policy_net_local.load_state_dict(torch.load(model_path+"policy.dat"))
                        self.model.policy_net_local.eval()
                        print('load model sucessfully')
