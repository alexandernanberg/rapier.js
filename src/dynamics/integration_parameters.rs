#[cfg(feature = "dim3")]
use rapier::dynamics::FrictionModel;
use rapier::dynamics::IntegrationParameters;
use wasm_bindgen::prelude::*;

/// The friction constraint model used by the solver.
///
/// Only 3D has more than one friction model, so this is not exposed in 2D.
#[cfg(feature = "dim3")]
#[wasm_bindgen]
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum RawFrictionModel {
    /// One Coulomb friction constraint per group of contacts, plus one twist
    /// constraint. Faster to solve than `Coulomb`, but less accurate.
    Simplified = 0,
    /// One Coulomb friction constraint per contact point.
    Coulomb = 1,
}

#[wasm_bindgen]
pub struct RawIntegrationParameters(pub(crate) IntegrationParameters);

#[wasm_bindgen]
impl RawIntegrationParameters {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        RawIntegrationParameters(IntegrationParameters::default())
    }

    #[wasm_bindgen(getter)]
    pub fn dt(&self) -> f32 {
        self.0.dt
    }

    #[wasm_bindgen(getter)]
    pub fn contact_erp(&self) -> f32 {
        self.0.contact_softness.erp(self.0.dt)
    }

    #[wasm_bindgen(getter)]
    pub fn normalizedAllowedLinearError(&self) -> f32 {
        self.0.normalized_allowed_linear_error
    }

    #[wasm_bindgen(getter)]
    pub fn normalizedPredictionDistance(&self) -> f32 {
        self.0.normalized_prediction_distance
    }

    #[wasm_bindgen(getter)]
    pub fn numSolverIterations(&self) -> usize {
        self.0.num_solver_iterations
    }

    #[wasm_bindgen(getter)]
    pub fn numInternalPgsIterations(&self) -> usize {
        self.0.num_internal_pgs_iterations
    }

    #[wasm_bindgen(getter)]
    pub fn maxCcdSubsteps(&self) -> usize {
        self.0.max_ccd_substeps
    }

    #[wasm_bindgen(getter)]
    pub fn lengthUnit(&self) -> f32 {
        self.0.length_unit
    }

    #[wasm_bindgen(getter)]
    pub fn contactNaturalFrequency(&self) -> f32 {
        self.0.contact_softness.natural_frequency
    }

    #[wasm_bindgen(getter)]
    pub fn contactDampingRatio(&self) -> f32 {
        self.0.contact_softness.damping_ratio
    }

    #[wasm_bindgen(getter)]
    pub fn staticContactNaturalFrequency(&self) -> f32 {
        self.0.static_contact_softness.natural_frequency
    }

    #[wasm_bindgen(getter)]
    pub fn staticContactDampingRatio(&self) -> f32 {
        self.0.static_contact_softness.damping_ratio
    }

    #[wasm_bindgen(getter)]
    pub fn warmstartCoefficient(&self) -> f32 {
        self.0.warmstart_coefficient
    }

    #[wasm_bindgen(getter)]
    pub fn warmstartJoints(&self) -> bool {
        self.0.warmstart_joints
    }

    #[wasm_bindgen(getter)]
    pub fn minCcdDt(&self) -> f32 {
        self.0.min_ccd_dt
    }

    #[wasm_bindgen(getter)]
    pub fn normalizedMaxCorrectiveVelocity(&self) -> f32 {
        self.0.normalized_max_corrective_velocity
    }

    #[wasm_bindgen(getter)]
    pub fn normalizedMaxLinearVelocity(&self) -> f32 {
        self.0.normalized_max_linear_velocity
    }

    #[wasm_bindgen(getter)]
    pub fn numInternalStabilizationIterations(&self) -> usize {
        self.0.num_internal_stabilization_iterations
    }

    #[wasm_bindgen(getter)]
    pub fn contactClustering(&self) -> bool {
        self.0.contact_clustering
    }

    #[wasm_bindgen(getter)]
    pub fn contactRecycling(&self) -> bool {
        self.0.contact_recycling
    }

    #[wasm_bindgen(getter)]
    pub fn normalizedContactRecycleDistance(&self) -> f32 {
        self.0.normalized_contact_recycle_distance
    }

    #[wasm_bindgen(getter)]
    pub fn frictionInBiasPass(&self) -> bool {
        self.0.friction_in_bias_pass
    }

    #[cfg(feature = "dim3")]
    #[wasm_bindgen(getter)]
    pub fn frictionModel(&self) -> RawFrictionModel {
        match self.0.friction_model {
            FrictionModel::Simplified => RawFrictionModel::Simplified,
            FrictionModel::Coulomb => RawFrictionModel::Coulomb,
        }
    }

    #[wasm_bindgen(setter)]
    pub fn set_dt(&mut self, value: f32) {
        self.0.dt = value;
    }

    #[wasm_bindgen(setter)]
    pub fn set_normalizedAllowedLinearError(&mut self, value: f32) {
        self.0.normalized_allowed_linear_error = value
    }

    #[wasm_bindgen(setter)]
    pub fn set_normalizedPredictionDistance(&mut self, value: f32) {
        self.0.normalized_prediction_distance = value
    }

    #[wasm_bindgen(setter)]
    pub fn set_numSolverIterations(&mut self, value: usize) {
        self.0.num_solver_iterations = value;
    }
    #[wasm_bindgen(setter)]
    pub fn set_numInternalPgsIterations(&mut self, value: usize) {
        self.0.num_internal_pgs_iterations = value;
    }
    #[wasm_bindgen(setter)]
    pub fn set_maxCcdSubsteps(&mut self, value: usize) {
        self.0.max_ccd_substeps = value
    }

    #[wasm_bindgen(setter)]
    pub fn set_lengthUnit(&mut self, value: f32) {
        self.0.length_unit = value
    }

    #[wasm_bindgen(setter)]
    pub fn set_contactNaturalFrequency(&mut self, value: f32) {
        self.0.contact_softness.natural_frequency = value
    }

    #[wasm_bindgen(setter)]
    pub fn set_contactDampingRatio(&mut self, value: f32) {
        self.0.contact_softness.damping_ratio = value
    }

    #[wasm_bindgen(setter)]
    pub fn set_staticContactNaturalFrequency(&mut self, value: f32) {
        self.0.static_contact_softness.natural_frequency = value
    }

    #[wasm_bindgen(setter)]
    pub fn set_staticContactDampingRatio(&mut self, value: f32) {
        self.0.static_contact_softness.damping_ratio = value
    }

    #[wasm_bindgen(setter)]
    pub fn set_warmstartCoefficient(&mut self, value: f32) {
        self.0.warmstart_coefficient = value
    }

    #[wasm_bindgen(setter)]
    pub fn set_warmstartJoints(&mut self, value: bool) {
        self.0.warmstart_joints = value
    }

    #[wasm_bindgen(setter)]
    pub fn set_minCcdDt(&mut self, value: f32) {
        self.0.min_ccd_dt = value
    }

    #[wasm_bindgen(setter)]
    pub fn set_normalizedMaxCorrectiveVelocity(&mut self, value: f32) {
        self.0.normalized_max_corrective_velocity = value
    }

    #[wasm_bindgen(setter)]
    pub fn set_normalizedMaxLinearVelocity(&mut self, value: f32) {
        self.0.normalized_max_linear_velocity = value
    }

    #[wasm_bindgen(setter)]
    pub fn set_numInternalStabilizationIterations(&mut self, value: usize) {
        self.0.num_internal_stabilization_iterations = value
    }

    #[wasm_bindgen(setter)]
    pub fn set_contactClustering(&mut self, value: bool) {
        self.0.contact_clustering = value
    }

    #[wasm_bindgen(setter)]
    pub fn set_contactRecycling(&mut self, value: bool) {
        self.0.contact_recycling = value
    }

    #[wasm_bindgen(setter)]
    pub fn set_normalizedContactRecycleDistance(&mut self, value: f32) {
        self.0.normalized_contact_recycle_distance = value
    }

    #[wasm_bindgen(setter)]
    pub fn set_frictionInBiasPass(&mut self, value: bool) {
        self.0.friction_in_bias_pass = value
    }

    #[cfg(feature = "dim3")]
    #[wasm_bindgen(setter)]
    pub fn set_frictionModel(&mut self, value: RawFrictionModel) {
        self.0.friction_model = match value {
            RawFrictionModel::Simplified => FrictionModel::Simplified,
            RawFrictionModel::Coulomb => FrictionModel::Coulomb,
        }
    }
}
