package org.firstinspires.ftc.teamcode.framework.util;

import java.util.concurrent.Callable;

public class SubsytemState {

    private String name;
    private Callable<String> run;


    public SubsytemState(String name, Callable<String> run) {
        this.name = name;
        this.run = run;
    }

    public String call() throws Exception {
        return run.call();
    }

    public String getName() {
        return this.name;
    }


}
