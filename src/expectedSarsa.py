import random

class ExpectedSarsa:
    def __init__(self, actions, epsilon, alpha, gamma):
        self.q = {}

        self.epsilon = epsilon
        self.alpha = alpha
        self.gamma = gamma
        self.actions = actions

    def getQ(self, state, action):
        return self.q.get((state, action), 0.0)

    def learnQ(self, state, action, reward, value):
        oldv = self.q.get((state, action), None)
        if oldv is None:
            self.q[(state, action)] = reward 
        else:
            self.q[(state, action)] = oldv + self.alpha * (value - oldv)

    def chooseAction(self, state):
        if random.random() < self.epsilon:
            action = random.choice(self.actions)
        else:
            q = [self.getQ(state, a) for a in self.actions]
            maxQ = max(q)
            count = q.count(maxQ)
            if count > 1:
                best = [i for i in range(len(self.actions)) if q[i] == maxQ]
                i = random.choice(best)
            else:
                i = q.index(maxQ)

            action = self.actions[i]
        return action

    def learn(self, state1, action1, reward, state2):
        exp_q = 0
	q = [self.getQ(state2, a) for a in self.actions]
        best = [i for i in range(len(self.actions)) if q[i] == max(q)] 			#calculate number of greedy actions
	
	non_greedy_actions_prob = (self.epsilon / len(self.actions))
        greedy_actions_prob = ((1 - self.epsilon) / len(best)) + (self.epsilon / len(self.actions))	

	for a in self.actions:
            if self.getQ(state2, a) == max(q): 						# This is a greedy action
                exp_q += self.getQ(state2, a) * greedy_actions_prob
            else: 									# This is a non-greedy action
                exp_q += self.getQ(state2, a) * non_greedy_actions_prob
	
        self.learnQ(state1, action1, reward, reward + self.gamma * exp_q)
