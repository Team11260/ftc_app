package org.firstinspires.ftc.teamcode.framework.util;

import java.util.concurrent.ConcurrentHashMap;

public class SubsystemStateMachine {

    private ConcurrentHashMap<String, SubsytemState> states;
    private String currentState = "";

    public SubsystemStateMachine() {
        states = new ConcurrentHashMap<>();
    }

    public void RunUntil(String start, String end) {

    }

    public void update() throws Exception {
        SubsytemState c = null;
        synchronized (currentState) {
            c = states.get(currentState);
            if (c == null)
                throw new StateConfigurationException("State not found: " + currentState);
        }

        currentState = c.call();
    }

    public void setCurrentState(String currentState) {
        synchronized (currentState) {
            this.currentState = currentState;
        }
    }

    public String getCurrentState() {
        synchronized (currentState) {
            return currentState;
        }
    }

    public void addState(SubsytemState... states) {
        for (SubsytemState state : states) {
            this.states.put(state.getName(), state);
        }
    }

}
