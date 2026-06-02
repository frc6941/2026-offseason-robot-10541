package lib.ironpulse.math.rbd;

public class TransformLookupException extends RuntimeException {
    public TransformLookupException(String target, String reference) {
        super("Cannot find transform with to: " + target + " and from: " + reference + " .");
    }
}
