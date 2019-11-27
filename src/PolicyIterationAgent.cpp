//
// Created by Chi Zhang on 8/26/19.
//

#include "PolicyIterationAgent.hpp"
#include "common.hpp"
#include <cmath>

PolicyIterationAgent::PolicyIterationAgent(const FrozenLakeMDP &mdp, double gamma, int iterations, double threshold) : ValueEstimateAgent(gamma, iterations, threshold), m_mdp(mdp)
{
    MSG("Training Policy Iteration Agent on " << m_mdp.getName());
    MSG("Initializing Policy Iteration Agent");
    initialize();
    MSG("Solving...");
    solve();
}

double PolicyIterationAgent::getQValue(const GameState &state, const Action &action)
{
    // TODO
    return qValueMap[std::make_pair(state, action)];
}

double PolicyIterationAgent::getValue(const GameState &state)
{
    // TODO
    return evaluateCurrentPolicy()[state];
}

Action PolicyIterationAgent::getPolicy(const GameState &state)
{
    // TODO
    return m_policy[state];
}

/*
 * Evaluate the current policy by returning V(s), which is represented as a map,
 * where key is GameState and value is double.
 */
std::map<GameState, double> PolicyIterationAgent::evaluateCurrentPolicy()
{

    std::map<GameState, double> val_map(valueMap);
    double delta = INT_MAX;
    int iteration = 0;
    while ((delta > m_threshold) && iteration <= m_iterations)
    {
        iteration++; // increment iteration

        std::set<GameState> possStates = m_mdp.getStates(); // get all possible states to iterate through
        std::set<GameState>::iterator state_iterator = possStates.begin();

        while (state_iterator != possStates.end())
        {
            // for each possible state in environment
            GameState curr_state = *state_iterator;
            double temp = valueMap[curr_state];

            std::vector<Action> possActions = m_mdp.getPossibleActions(curr_state); // get all possible actions
            if (possActions.size() == 0)
            {
                state_iterator++;
                continue;
            }
            Action optimalAction = LEFT;
            double maxActionValue = INT_MIN;

            for (Action action : possActions)
            {
                // for each possible action, get all possible transition states and probabilities
                std::map<GameState, double> transitionProbs = m_mdp.getTransitionStatesAndProbs(curr_state, action);
                std::map<GameState, double>::iterator transition_iterator = transitionProbs.begin();

                double sum = 0; // sum for current action

                while (transition_iterator != transitionProbs.end())
                {
                    // for each possible transition state (w/ probability)
                    std::pair<GameState, double> transition_pair = *transition_iterator;
                    // val = prob(s' | s, a) * [R(s, a, s') + γ(v[s'])]
                    sum += transition_pair.second * (m_mdp.getReward(curr_state, action, transition_pair.first) + (m_gamma * valueMap[transition_pair.first]));

                    transition_iterator++; // iterate to next possible transition state
                }

                // insert q value into map
                // qValueMap.insert(std::make_pair(std::make_pair(curr_state, action), sum));
                qValueMap[std::make_pair(curr_state, action)] = sum;

                // pick optimal action based on current max value
                if (sum > maxActionValue)
                {
                    maxActionValue = sum;
                }

                // v value is larger of current sum (for current action) or current v value
            }
            // valueMap[curr_state] = v; // set value for V[s] to newly calculated v value
            val_map[curr_state] = maxActionValue;

            // delta = std::max(delta, std::abs(temp - v));
            // delta = std::max(delta, std::abs(temp - maxActionValue));

            ++state_iterator; // iterate to next state in environment
        }
    }

    return val_map;
}

void PolicyIterationAgent::solve()
{
    // TODO
    bool stable = true;
    while (stable)
    {
        valueMap = evaluateCurrentPolicy();

        std::set<GameState> possStates = m_mdp.getStates();
        std::set<GameState>::iterator state_iterator = possStates.begin();

        while (state_iterator != possStates.end()) // for each state s in S
        {
            GameState curr_state = *state_iterator;
            std::map<GameState, Action> temp(m_policy); // copy policy map to temp variable

            std::vector<Action> possActions = m_mdp.getPossibleActions(curr_state); // get all possible actions
            if (possActions.size() == 0)
            {
                state_iterator++;
                continue;
            }

            Action optimalAction = LEFT;
            double maxActionValue = INT_MIN;

            for (Action action : possActions)
            {
                // for each possible action, get all possible transition states and probabilities
                std::map<GameState, double> transitionProbs = m_mdp.getTransitionStatesAndProbs(curr_state, action);
                std::map<GameState, double>::iterator transition_iterator = transitionProbs.begin();

                double sum = 0; // sum for current action

                while (transition_iterator != transitionProbs.end())
                {
                    // for each possible transition state (w/ probability)
                    std::pair<GameState, double> transition_pair = *transition_iterator;
                    // val = prob(s' | s, a) * [R(s, a, s') + γ(v[s'])]
                    sum += transition_pair.second * (m_mdp.getReward(curr_state, action, transition_pair.first) + (m_gamma * valueMap[transition_pair.first]));

                    transition_iterator++; // iterate to next possible transition state
                }

                // pick optimal action based on current max value
                if (sum > maxActionValue)
                {
                    maxActionValue = sum;
                    optimalAction = action;
                }

                m_policy[curr_state] = optimalAction; // set π[s] = argmax_a
            }

            if (m_policy[curr_state] != temp[curr_state])
            {
                stable = false;
            }

            ++state_iterator; // iterate to next state in environment
        }
    }
}

void PolicyIterationAgent::initialize()
{
    // TODO
    // valueMap --> initialize to values of 0 for all possible game states?
    std::set<GameState> possStates = m_mdp.getStates();
    std::set<GameState>::iterator it = possStates.begin();
    while (it != possStates.end())
    {
        // for all possible states, initialize value(S) to 0
        GameState state = *it;
        m_policy[state] = LEFT;
        valueMap[state] = 0;
        ++it;
    }
}
