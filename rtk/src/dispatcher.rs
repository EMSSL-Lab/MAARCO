use crate::protocol::response::WireMessage;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::Sender;
use std::sync::{Arc, Mutex};

type MatchFn = Box<dyn Fn(&WireMessage) -> bool + Send + Sync>;

struct Waiter {
    id: u64,
    /// Function to determine if a message matches the waiter's criteria.
    matches: MatchFn,
    /// Transmitter to send matched messages to the waiter.
    tx: Sender<WireMessage>,
    /// How many messages are still needed before the waiter is satisfied:
    remaining: u32,
}

#[derive(Clone)]
pub struct Dispatcher {
    /// The transmitter for the main data stream. Used to forward messages not claimed by waiters.
    stream_tx: Option<Sender<WireMessage>>,
    /// The list of waiters listening for specific messages.
    waiters: Arc<Mutex<Vec<Waiter>>>,
    next_waiter_id: Arc<AtomicU64>,
}

impl Dispatcher {
    pub fn new() -> Self {
        Dispatcher {
            stream_tx: None,
            waiters: Arc::new(Mutex::new(Vec::new())),
            next_waiter_id: Arc::new(AtomicU64::new(1)),
        }
    }

    pub fn set_stream_tx(&mut self, tx: Sender<WireMessage>) {
        self.stream_tx = Some(tx);
    }

    /// Registers a new waiter with a matching function and a transmitter.
    pub fn register_waiter(&self, matches: MatchFn, tx: Sender<WireMessage>, count: u32) -> u64 {
        let id = self.next_waiter_id.fetch_add(1, Ordering::Relaxed);
        let mut waiters = self.waiters.lock().unwrap();
        waiters.push(Waiter {
            id,
            matches,
            tx,
            remaining: count,
        });
        id
    }

    /// Removes a waiter after its caller times out or fails to write.
    pub fn remove_waiter(&self, id: u64) {
        self.waiters
            .lock()
            .unwrap()
            .retain(|waiter| waiter.id != id);
    }

    pub fn dispatch(&self, msg: WireMessage) {
        // Try to satisfy waiters first:
        let mut waiters = self.waiters.lock().unwrap();
        let mut i = 0;

        // println!("Waiters count: {}", waiters.len());
        while i < waiters.len() {
            if (waiters[i].matches)(&msg) {
                // Send the message to the waiter:
                // println!("A waiter claimed the message {:?}, sending to waiter.", &msg);
                if waiters[i].tx.send(msg.clone()).is_err() {
                    // The receiver timed out and was dropped. Remove the stale
                    // waiter, then offer this same message to newer waiters.
                    waiters.remove(i);
                    continue;
                }
                // Decrement remaining count
                waiters[i].remaining -= 1;

                // Remove if done
                if waiters[i].remaining == 0 {
                    waiters.remove(i);
                }
                drop(waiters); // Release lock
                return;
            } else {
                i += 1;
            }
        }
        drop(waiters); // Release lock
        // No waiter claimed the message, send to stream:
        // println!("No waiter claimed the message, sending to stream.");
        self.fanout_stream(msg);
    }

    fn fanout_stream(&self, msg: WireMessage) {
        if let Some(ref tx) = self.stream_tx {
            // println!("Sending message to stream.");
            let _ = tx.send(msg);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::response::PQTMResponse;
    use std::sync::mpsc;

    #[test]
    fn stale_waiter_does_not_consume_message_for_live_waiter() {
        let dispatcher = Dispatcher::new();
        let (stale_tx, stale_rx) = mpsc::channel();
        drop(stale_rx);
        dispatcher.register_waiter(Box::new(|_| true), stale_tx, 1);

        let (live_tx, live_rx) = mpsc::channel();
        dispatcher.register_waiter(Box::new(|_| true), live_tx, 1);

        dispatcher.dispatch(WireMessage::PQTMMessage(PQTMResponse::SaveParOk));

        assert!(matches!(
            live_rx.try_recv(),
            Ok(WireMessage::PQTMMessage(PQTMResponse::SaveParOk))
        ));
    }

    #[test]
    fn waiter_can_be_removed_after_timeout() {
        let dispatcher = Dispatcher::new();
        let (tx, _rx) = mpsc::channel();
        let id = dispatcher.register_waiter(Box::new(|_| true), tx, 1);

        dispatcher.remove_waiter(id);

        assert!(dispatcher.waiters.lock().unwrap().is_empty());
    }
}
