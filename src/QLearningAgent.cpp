//
// Created by Chi Zhang on 8/27/19.
//

#include "common.hpp"
#include "third_party/random.hpp"
#include "QLearningAgent.hpp"
#include <iostream>
#include <fstream>
#include <queue>

QLearningAgent::QLearningAgent(FrozenLakeEnv &env, double gamma, int iterations, double alpha, double epsilon) : ValueEstimateAgent(gamma, iterations, 0.0), m_alpha(alpha), m_epsilon(epsilon), m_env(env)
{
    MSG("Training Q Learning Agent on " << m_env.getName());
    MSG("Initializing Q Learning Agent");
    initialize();
    MSG("Solving...");
    solve();
}

std::pair<Action, double> QLearningAgent::getMaxActionValue(const GameState &state)
{
    double maxQ = INT_MIN;
    Action maxAction = LEFT;

    std::vector<Action> possActions = m_env.getPossibleActions(state);
    if (m_env.isTerminal(state))
        return std::make_pair(LEFT, 0);
    for (Action poss : possActions)
    {
        if (getQValue(state, poss) > maxQ)
        {
            maxAction = poss;
            maxQ = getQValue(state, poss);
        }
    }
    return std::make_pair(maxAction, maxQ);
}

double QLearningAgent::getValue(const GameState &state)
{
    return getMaxActionValue(state).second;
}

double QLearningAgent::getQValue(const GameState &state, const Action &action)
{
    // * IF TERMINAL NODE
    if (m_env.isTerminal(state))
        return 0.0;

    if (m_qvalue.find(std::make_pair(state, action)) == m_qvalue.end())
    {
        // * IF Q[S,A] DOES NOT EXIST
        // * SUBSTITUTE 0
        m_qvalue[std::make_pair(state, action)] = 0;
    }

    return m_qvalue[std::make_pair(state, action)];
}

// The final policy without exploration. Used for evaluation.
Action QLearningAgent::getPolicy(const GameState &state)
{
    return getMaxActionValue(state).first;
}

// you should use getAction in solve instead of getPolicy and implement your exploration strategy here.
Action QLearningAgent::getAction(const GameState &state)
{

    // ! COUNTING BASED EXPLORATION
    // std::vector<Action> possActions = m_env.getPossibleActions(state);
    // Action minUsedAction;
    // int minUsedCount = -1;
    // for (Action poss : possActions)
    // {

    //     if (m_nvalue.find(std::make_pair(state, poss)) == m_nvalue.end())
    //     {
    //         m_nvalue[std::make_pair(state, poss)] = 1;
    //         return poss;
    //     }

    //     int count = m_nvalue[std::make_pair(state, poss)];
    //     if (count < minUsedCount)
    //     {
    //         minUsedCount = count;
    //         minUsedAction = poss;
    //     }
    // }
    // return minUsedAction;
    return getPolicy(state);

    // ! EPSILON-GREEDY ACTION SELECTION
    /* std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist_double(0.00, 1.00);

    std::vector<Action> possActions = m_env.getPossibleActions(state);
    std::uniform_int_distribution<int> dist_int(0, possActions.size() - 1);

    if (possActions.size() == 0)
        return LEFT;

    double probability = dist_double(gen);
    int actionToChoose = dist_int(gen);

    Action action = possActions[actionToChoose];

    if (probability < m_epsilon)
    {
        std::cout << "action to choose before loop: " << actionToChoose << std::endl;
        while (action == getMaxActionValue(state).first)
        {
            actionToChoose = dist_int(gen);
            action = possActions[actionToChoose];
        }

        return action;
    }

    else // if probability > m_epsilon, pick current best option (m_policy)
    {
        return getPolicy(state);
    }

    return LEFT; // control will never reach this point */
}

void QLearningAgent::update(const GameState &state, const Action &action, const GameState &nextState, double reward)
{
    // * COUNTING BASED EXPLORATION
    // * q′(s,a) = q(s,a)+βN(s,a)^(-1/2)
    double currQValue = getQValue(state, action);

    m_qvalue[std::make_pair(state, action)] = m_qvalue[std::make_pair(state, action)] + (m_epsilon * pow(m_nvalue[std::make_pair(state, action)], -1 / 2));

    std::pair<Action, double> nextQValue = getMaxActionValue(nextState);

    // * Q[s, a] = Q[s, a] + alpha[reward + gamma(max_a'(Q[s', a'])) - Q[s, a]]
    // ? IF BOTH Q[S,A] and Q[S', A'] EXIST
    m_qvalue[std::make_pair(state, action)] = currQValue + m_alpha * (reward + (m_gamma * (nextQValue.second) - m_qvalue[std::make_pair(state, action)]));

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
        GameState state = m_env.reset(); // reset environment and get initial state
        while (!m_env.isTerminal(state))
        {
            Action action = getAction(state); // use instead of getPolicy?
            GameState nextState = m_env.getNextState(state, action);
            double reward = m_env.getReward(state, action, nextState);

            // * DEBUG STATEMENTS
            // std::cout << "EPISODE: " << numSteps << std::endl;
            // std::cout << "state: " << state.getLoc().x << " " << state.getLoc().y
            //           << " action: " << action
            //           << " nextState: " << nextState.getLoc().x << " " << nextState.getLoc().y
            //           << " reward: " << reward << std::endl;

            update(state, action, nextState, reward);
            m_nvalue[std::make_pair(state, action)]++; // increment N(s, a)
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
    // * Q LEARNING INITIALIZES AS IT EXPLORES --> NO INITIALIZE() FUNCTION NECESSARY
    return;
}
