//
// Created by Chi Zhang on 8/26/19.
//

#include "PolicyIterationAgent.hpp"
#include "common.hpp"
#include <cmath>

PolicyIterationAgent::PolicyIterationAgent(const FrozenLakeMDP &mdp, double gamma, int iterations, double threshold) :
        ValueEstimateAgent(gamma, iterations, threshold), m_mdp(mdp) {
    MSG("Training Policy Iteration Agent on " << m_mdp.getName());
    MSG("Initializing Policy Iteration Agent");
    initialize();
    MSG("Solving...");
    solve();
}

double PolicyIterationAgent::getQValue(const GameState &state, const Action &action) {
    // TODO
    return 0.0;
}

double PolicyIterationAgent::getValue(const GameState &state) {
    // TODO
    return 0.0;
}

Action PolicyIterationAgent::getPolicy(const GameState &state) {
    // TODO
    return LEFT;
}

/*
 * Evaluate the current policy by returning V(s), which is represented as a map,
 * where key is GameState and value is double.
 */
std::map<GameState, double> PolicyIterationAgent::evaluateCurrentPolicy() {
    // TODO
    return std::map<GameState, double>();
}

void PolicyIterationAgent::solve() {
    // TODO
}

void PolicyIterationAgent::initialize() {
    // TODO
}


