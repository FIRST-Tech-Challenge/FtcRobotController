package com.technototes.subsystem;

/** An interface for indexers on robot
 * @author Alex Stedman
 */
public interface IndexerSubsystem extends IntakeSubsystem {
    /** Empty one item from the indexer into its destination
     *
     * @param num The number to dispense
     */
    void load(int num);

    /** Load one item from the indexer to destination
     *
     */
    default void loadOne(){
        load(1);
    }

    /** Spit out/extake all items from indexer
     *
     */
    default void spit(){
        extake();
    }
    /** Load all items from the indexer to destination
     *
     */
    default void loadAll(){
        intake();
    }
}
