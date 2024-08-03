use vexide_async::*;

pub trait State<O> {
    async fn update(&mut self) -> Option<O>;
}

pub trait Subsystem<O> {
    async fn run(&mut self, state: impl State<O>);
}
