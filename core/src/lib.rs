use nalgebra::Vector3;

pub mod filter;

/// Motor output containing the desired pitch, roll, and yaw control and feed forward in -1 ~ +1.
#[derive(Debug)]
pub struct MotorOutput<T> {
    pub  control: Vector3<T>,
     pub feed_forward: Vector3<T>
 }
 
 impl<T> MotorOutput<T> {
     pub fn new(control: Vector3<T>, feed_forward: Vector3<T>) -> Self {
         Self {
             control,
             feed_forward
         }
     }
 }
 

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
