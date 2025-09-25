Due to Proteus limitation, I recommend setting durations for timer interrupts higher than 5 unit (e.g dur2 = 5; - 50ms).

What would happen if the duration was smaller than 5 (let's say 1)? 
It is fine, that what the hardware should be able to handle.
But Proteus's simulation might be broken. It is not guaranteed to illustrate expected outcome.