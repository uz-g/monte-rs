pub trait State<I, O> {
    fn init(&mut self) {}
    fn update(&mut self, i: &I) -> Option<O>;
}

pub trait Subsystem<I, O> {
    async fn run(&mut self, state: impl State<I, O>);
}
