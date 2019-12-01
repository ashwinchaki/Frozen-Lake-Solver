//
// Created by Chi Zhang on 8/24/19.
//

#include "common.hpp"
#include "ValueIterationAgent.hpp"

ValueIterationAgent::ValueIterationAgent(FrozenLakeMDP const &mdp, double gamma, int iterations, double threshold) : ValueEstimateAgent(gamma, iterations, threshold), m_mdp(mdp)
{
    MSG("Training Value Iteration Agent on " << m_mdp.getName());
    MSG("Initializing Value Iteration Agent");
    initialize();
    MSG("Solving...");
    solve();
}

double ValueIterationAgent::getValue(const GameState &state)
{
    return valueMap[state];
}

double ValueIterationAgent::getQValue(const GameState &state, const Action &action)
{
    return qValueMap[std::make_pair(state, action)];
}

Action ValueIterationAgent::getPolicy(const GameState &state)
{
    return policyMap[state];
}

void ValueIterationAgent::solve()
{
    double delta = INT_MAX;
    bool d = false;
    int iteration = 0;
    while (!d && iteration <= m_iterations)
    {
        d = true;
        iteration++; // increment iteration

        std::set<GameState> possStates = m_mdp.getStates(); // get all possible states to iterate through
        std::set<GameState>::iterator state_iterator = possStates.begin();

        while (state_iterator != possStates.end())
        {
            // for each possible state in environment
            GameState curr_state = *state_iterator;
            double temp = valueMap[curr_state];
            double v = INT_MIN;

            std::vector<Action> possActions = m_mdp.getPossibleActions(curr_state); // get all possible actions
            if (possActions.size() == 0)
            {
                state_iterator++;
                continue;
            }
            Action optimalAction = LEFT;
            double maxActionValue = qValueMap[std::make_pair(curr_state, optimalAction)];

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
                qValueMap[std::make_pair(curr_state, action)] = sum;

                // pick optimal action based on current max value
                if (sum > maxActionValue)
                {
                    maxActionValue = sum;
                    optimalAction = action;
                }

                // v value is larger of current sum (for current action) or current v value
                if (sum - valueMap[curr_state] > m_threshold)
                    d = false;
                v = std::max(sum, v);
            }
            policyMap[curr_state] = optimalAction;

            valueMap[curr_state] = maxActionValue;

            // delta = std::max(delta, std::abs(temp - maxActionValue));

            ++state_iterator; // iterate to next state in environment
        }
    }
}

void ValueIterationAgent::initialize()
{
    // TODO. Initialize your data structure.

    // valueMap --> initialize to values of 0 for all possible game states?
    std::set<GameState> possStates = m_mdp.getStates();
    std::set<GameState>::iterator it = possStates.begin();
    while (it != possStates.end())
    {
        // for all possible states, initialize value(S) to 0
        GameState state = *it;
        valueMap[state] = 0;
        ++it;
    }
}
