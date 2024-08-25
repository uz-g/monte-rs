pub trait State<I, O> {
    fn init(&mut self) {}
    fn update(&mut self, i: &I) -> Option<O>;
}
