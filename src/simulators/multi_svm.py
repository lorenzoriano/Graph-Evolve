from sklearn import grid_search
import numpy as np

class MultiSVR(object):
    def __init__(self, models):
        self.models = models

    def train(self, model, parameters, trainX, trainY):
        self.models = []
        for i in xrange(trainY.shape[1]):
            print "trainig model %d" % i
            grid = grid_search.GridSearchCV(model, parameters)
            grid.fit(trainX, trainY[:,i])
            self.models.append(grid.best_estimator)

    def score(self, dataX, dataY):
        scores = []
        for i in xrange(dataY.shape[1]):
            score = self.models[i].score(dataX, dataY[:,i])
            scores.append(score)
        return scores

    def predict(self, dataX):
        outs = []
        for m in self.models:
            outs.append(m.predict(dataX))

        return np.array(outs).T
