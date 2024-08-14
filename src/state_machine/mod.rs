pub trait State<I, O> {
    async fn init(&mut self) {}
    async fn update(&mut self, i: &I) -> Option<O>;
}

pub trait Subsystem<I, O> {
    async fn run(&mut self, state: impl State<I, O>);
}
