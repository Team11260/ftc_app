package org.firstinspires.ftc.teamcode.framework.util;

import android.support.annotation.NonNull;

import org.firstinspires.ftc.teamcode.framework.abstractopmodes.AbstractOpMode;

import java.util.ArrayList;
import java.util.Map;
import java.util.concurrent.Callable;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

// Emitter is event handling middleware.
//
// Create a new event with 'emit'. Register event handlers with 'on'.
public class Emitter {
    private ConcurrentHashMap<String, ArrayList<Callable<Boolean>>> eventRegistry = new ConcurrentHashMap<>();
    private ArrayList<String> pausedEvents = new ArrayList<>();
    private ArrayList<Future> runningFutures = new ArrayList<>();
    private ConcurrentHashMap<String, ArrayList<Future<Boolean>>> cache = new ConcurrentHashMap<>();

    private ExecutorService service;

    public Emitter() {
        service = Executors.newCachedThreadPool();
    }

    // Register a new event handler.
    // An event handler is a Runnable that is run on an Executor.
    public synchronized void on(String eventName, Callable eventHandler) {
        if(eventRegistry.get(eventName) == null) eventRegistry.put(eventName, new ArrayList<>());
        eventRegistry.get(eventName).add(eventHandler);
    }

    public synchronized void pauseEvent(String name) {
        pausedEvents.add(name);
    }

    public synchronized void resumeEvent(String name) {
        pausedEvents.remove(name);
    }

    public synchronized void removeEvent(String name) {
        eventRegistry.remove(name);
    }

    public synchronized void update() {

        ArrayList<Future> finishedFutures = new ArrayList<>();

        for(Future<Boolean> future:runningFutures){
            if(future.isDone()){
                try {
                    future.get();
                } catch (InterruptedException e) {
                    AbstractOpMode.staticThrowException(e);
                } catch (ExecutionException e) {
                    AbstractOpMode.staticThrowException(e);
                }
                finishedFutures.add(future);
            }
        }

        runningFutures.removeAll(finishedFutures);
    }

    // Send a named event.
    //
    // This will run all event handlers registered to this event. Each event handler will be
    // executed inside of the executor service, which means events may be handled in parallel.
    public synchronized void emit(String name) throws RuntimeException {

        for (String pausedName : pausedEvents) {
            if (pausedName.equals(name)) return;
        }

        ArrayList<Future<Boolean>> f = fire(name);
        cache.put(name, f);
        runningFutures.addAll(f);
    }

    public synchronized ArrayList<Future<Boolean>> futuresFor(String eventName) {
        if (this.cache.contains(eventName)) {
            return this.cache.get(eventName);
        }
        return new ArrayList<>();
    }

    // This does the actual firing of the event. The emit() method calls this, and caches
    // the resulting future for cancellation.
    protected synchronized ArrayList<Future<Boolean>> fire(String eventName) throws RuntimeException {
        ArrayList<Future<Boolean>> futures = new ArrayList<>();
        for (Callable eventHandler : eventRegistry.get(eventName)) {
            if (eventHandler != null) futures.add(service.submit(eventHandler));
        }
        return futures;
    }

    public synchronized void shutdown() {
        for (ConcurrentHashMap.Entry<String, ArrayList<Future<Boolean>>> entry : cache.entrySet()) {
            for(Future<Boolean> future : entry.getValue()) {
                if (future.isDone()) {
                    future.cancel(true);
                }
            }
        }
        service.shutdownNow();
    }

    class EmptyResult implements Future<Boolean> {
        @Override
        public boolean cancel(boolean b) {
            return false;
        }

        @Override
        public boolean isCancelled() {
            return false;
        }

        @Override
        public boolean isDone() {
            return true;
        }

        @Override
        public Boolean get() throws InterruptedException, ExecutionException {
            return false;
        }

        @Override
        public Boolean get(long l, @NonNull TimeUnit timeUnit) throws InterruptedException, ExecutionException, TimeoutException {
            return false;
        }
    }
}
