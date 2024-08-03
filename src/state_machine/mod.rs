pub trait State<O> {
    async fn init(&mut self) {}
    async fn update(&mut self) -> Option<O>;
}

pub trait Subsystem<O> {
    async fn run(&mut self, state: impl State<O>);
}
