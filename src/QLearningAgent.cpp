//
// Created by Chi Zhang on 8/27/19.
//

#include "common.hpp"
#include "third_party/random.hpp"
#include "QLearningAgent.hpp"
#include <iostream>
#include <fstream>

QLearningAgent::QLearningAgent(FrozenLakeEnv &env, double gamma, int iterations, double alpha, double epsilon) : ValueEstimateAgent(gamma, iterations, 0.0), m_alpha(alpha), m_epsilon(epsilon), m_env(env)
{
    MSG("Training Q Learning Agent on " << m_env.getName());
    MSG("Initializing Q Learning Agent");
    initialize();
    MSG("Solving...");
    solve();
}

double QLearningAgent::getValue(const GameState &state)
{
    // TODO
    return 0.0;
}

double QLearningAgent::getQValue(const GameState &state, const Action &action)
{
    // TODO
    return 0.0;
}

// The final policy without exploration. Used for evaluation.
Action QLearningAgent::getPolicy(const GameState &state)
{
    // TODO
    return LEFT;
}

// you should use getAction in solve instead of getPolicy and implement your exploration strategy here.
Action QLearningAgent::getAction(const GameState &state)
{
    // TODO
    return LEFT;
}

void QLearningAgent::update(const GameState &state, const Action &action, const GameState &nextState, double reward)
{
    // TODO
    return;
}

void QLearningAgent::solve()
{
    // output a file for plotting
    std::ofstream outFile;
    outFile.open("result.csv");
    outFile << "Episode,Reward" << std::endl;

    int maxEpisodeSteps = 100;
    // collect m_iterations trajectories for update
    for (int i = 0; i < m_iterations; i++)
    {
        int numSteps = 0;
        GameState state = m_env.reset();
        while (!m_env.isTerminal(state))
        {
            Action action = getAction(state);
            GameState nextState = m_env.getNextState(state, action);
            double reward = m_env.getReward(state, action, nextState);

            update(state, action, nextState, reward);
            state = nextState;
            numSteps += 1;
            if (numSteps >= maxEpisodeSteps)
                break; // avoid infinite loop in some cases.
        }
        // evaluate for 100 episodes using the current optimal policy. You can't change this line.
        double episodeReward = m_env.runGame(*this, 100, m_gamma, false).first;
        std::cout << "Evaluating episode reward at learning iteration " << i << " is " << episodeReward << std::endl;
        outFile << i << "," << episodeReward << std::endl;
    }
    outFile.close();
}

void QLearningAgent::initialize()
{
    // TODO
    return;
}
